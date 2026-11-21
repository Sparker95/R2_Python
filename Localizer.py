# -*- coding: utf-8 -*-
"""
Created on Mon Apr 28 21:46:59 2025

@author: spark
"""

import numpy as np
import scipy.stats

import Utils
import Map
import BresenhamLine
import Chassis
import Scan

# Interface between localizer and world & sensor models (map and sensor model)
class LocalizerWorldInterfaceBase:
    
    # Returns xmin, ymin, xmax, ymax of world
    def GetWorldBounds():
        return (0, 0, 0, 0)
    
    # Returns True if given pos (x,y) is valid in the world (not inside object)
    def IsPoseValid(self, pose):
        return True
    
    # Traces line through world, returns expected distance until hit, -1 if no hit is expected
    def GetScan(self, pose, map, scanAngles):
        return np.zeros((2,10), dtype=float)
    
    # Returns max range of sensor scan
    def GetScanRange(self):
        return float(10)
    
# END CLASS

# struct Config
# {
# 	float m_SensorStdDev = 0.1f;
# 	Pose m_SensorOffset_LS = {};
# 	float m_SensorScanRange = 5.0f;
# 	float m_MapRes_w = 0.125f;
# 	Pose m_StartPose = {};
# 	uint m_nParticles = 64;
# 	int m_L2IntOccupiedCell = 1; // Use Utils::L2FloatToL2Int()
# 	int m_L2IntFreeCell = 1;
# };

# class LocalizerConfig:
#     def __init__(self):
#         self.m_sensorStdDev = 0.1
#         self.m_sensorOffset_LS = (float(0), float(0), float(0))
        

class Localizer:
    
    """
    Particles array:
    0 - x
    1 - y
    2 - dir
    3 - weight
    4 - map
    5 - history
    """
    
    # sensorOffset - (fwd(along X axis), side(along Y axis), dir)
    def __init__(self, startPose, nParticles, sensorStdDev, worldInterface, chassisStatsModel, mapRes_w, sensorOffset, sensorPdfFloor, l2OccupiedCell, l2FreeCell):
        self.m_startPose = startPose
        self.m_worldInterface = worldInterface
        self.m_chassisModel = chassisStatsModel
        self.m_sensorStdDev = sensorStdDev # Standard deviation of one lidar ray
        self.m_sensorPdfFloor = sensorPdfFloor
        self.m_nParticles = int(nParticles)
        self.m_rngResample = np.random.default_rng(17)
        self.m_particlesHistory = []
        self.m_sensorOffset = (float(sensorOffset[0]), float(sensorOffset[1]), float(sensorOffset[2]))
        self.m_l2IntOccupiedCell = Utils.L2FloatToL2Int(l2OccupiedCell)
        self.m_l2FreeCell = Utils.L2FloatToL2Int(l2FreeCell)
        self.m_testErrors = []
        self.m_mapRes_w = mapRes_w
        
        # Init particles
        self.m_particles = []
        for i in range(self.m_nParticles):
            p = self.NewParticle(startPose, mapRes_w)
            self.m_particles.append(p)
        
    def NewParticle(self, startPose, mapRes_w):
        emptyMap = Map.Map(128, 128, mapRes_w) # New empty map
        # x, y, dir, weight, map, history
        return [float(startPose[0]), float(startPose[1]), float(startPose[2]), float(0), emptyMap, []]
    
    # Returns array(nParticles, 4): (x, y, dir, weight, map)
    def GetParticles(self):
        return self.m_particles
    
    # Returns pose with biggest weight
    def GetMostLikelyPose(self):
        p = self.GetMostLikelyParticle()
        return (p[0], p[1], p[2])
    
    
    def GetMostLikelyParticle(self):
        maxWeight = self.m_particles[0][3]
        maxId = 0
        for i in range(self.m_nParticles):
            if (self.m_particles[i][3] > maxWeight):
                maxWeight = self.m_particles[i][3]
                maxId = i
                
        return self.m_particles[maxId]
            
    def Update(self, chassisInputs, realScanPolar):
        
        chassisInputs_rnd = Chassis.ChassisInputs()
        for i in range(self.m_nParticles):  
            # Apply motion model to particle
            p = self.m_particles[i]
            particlePose = tuple(p[0:3])
            randomPoseDiff_LS = self.m_chassisModel.CalculateRandomPoseDiff_LS(chassisInputs, chassisInputs_rnd)
            poseRandomized = Utils.AddPoseDiff(particlePose, randomPoseDiff_LS)
            
            # Compensate scan motion
            scanCompensated = Scan.CompensateScanMotion(realScanPolar, self.m_chassisModel, chassisInputs_rnd)
            
            p[5].append(particlePose) # Push history of this particle
            p[0] = poseRandomized[0]
            p[1] = poseRandomized[1]
            p[2] = poseRandomized[2]
            
            # Calculate weight
            weight = self.CalculateParticleWeight(p, scanCompensated)
            p[3] = weight
            
            # Update map
            # Scan is applied at pose where Sensor is in World Space
            sensorPoseRandomized = Utils.AddPoseDiff(poseRandomized, self.m_sensorOffset)
            self.ApplyScan(p[4], scanCompensated, sensorPoseRandomized)
        
        self.NormalizeWeights()
        
        # Save history
        self.SaveParticlesHistory()
        
        # Resample particles, use value of effective particles
        sumWeightSq = float(0)
        for i in range(self.m_nParticles):
            sumWeightSq += (self.m_particles[i][3] * self.m_particles[i][3])
        nEff = 1/sumWeightSq
        
        if (nEff < self.m_nParticles/2):
            print(f"N effective particles: {nEff}, resampling")
            self.ResampleParticles()
        else:
            print(f"N effective particles: {nEff}, NO resampling")
        
        # Save history
        self.SaveParticlesHistory()
        
        # Run test
        bestParticle = self.GetMostLikelyParticle()
        bestParticlePose = (bestParticle[0], bestParticle[1], bestParticle[2])
        for i in range(realScanPolar.shape[1]):
            testMap = Map.Map(320, 320, self.m_mapRes_w, allocData = True)
            self.ApplyScan(testMap, realScanPolar, bestParticlePose)
            testScan = self.m_worldInterface.GetScan(bestParticlePose, testMap, realScanPolar[0][:])
            distDiff = (realScanPolar[1,i] - testScan[1,i]) / self.m_mapRes_w
            self.m_testErrors.append(distDiff)
        testMap = 0
        
    
    # Saves all current particles
    def SaveParticlesHistory(self):
        particlesCopy = []
        for i in range(self.m_nParticles):
            p = self.m_particles[i]
            pcopy = [p[0], p[1], p[2], p[3], 0, p[5].copy()] # Copy full history of this particle
            particlesCopy.append(pcopy)
        self.m_particlesHistory.append(particlesCopy)
    
    # Page 110
    def ResampleParticles(self):
        r = self.m_rngResample.uniform(0, 1 / self.m_nParticles)
        c = self.m_particles[0][3] # Weight 0
        i = int(0)
        newParticles = []
        
        for m in range(self.m_nParticles):
            u = r + m / self.m_nParticles
            while u > c:
                i = (i + 1) % self.m_nParticles
                c += self.m_particles[i][3]
            # Add particle i
            newParticles.append(self.CopyParticle(self.m_particles[i]))
        
        # Swap arrays
        self.m_particles = newParticles
        
    def CopyParticle(self, p):
        # Make full copy of map, full copy of history
        return [p[0], p[1], p[2], p[3], p[4].Copy(), p[5].copy()]
        
    def CalculateParticleWeight(self, particle, realScanPolar):
        # Scan is taken at Pose where Sensor is (would be) in Worls Space
        particleSensorPose = Utils.AddPoseDiff((particle[0], particle[1], particle[2]), self.m_sensorOffset)
        expectedScanPolar = self.m_worldInterface.GetScan(particleSensorPose, particle[4], realScanPolar[0][:])
        
        w = pow(2,96) # pow(2,32); It's big number because weights only decrease.
        for i in range(realScanPolar.shape[1]):
           _w = self.SensorRayPdf(realScanPolar[1,i], expectedScanPolar[1,i])
           w *= _w
        return w
    
    def SensorRayPdf(self, realDist, expectedDist):
        
        # If it doesn't hit anything, then any value is equally likely
        if (expectedDist == -1):
            return self.m_sensorPdfFloor
        
        diff = realDist - expectedDist
        diffNorm = diff / self.m_sensorStdDev
        diffNormPow2 = diffNorm * diffNorm
        diffNormPow4 = diffNormPow2 * diffNormPow2
        diffNormPow8 = diffNormPow4 * diffNormPow4
        pdfNoFloor = 1.0 / (diffNormPow8 + 1)
        
        totalPdf = self.m_sensorPdfFloor + (1 - self.m_sensorPdfFloor) * pdfNoFloor
        return totalPdf
    
    def ApplyScan(self, map, scanPolar, pose):
        (startx_w, starty_w, startDir_w) = pose
        
        # No-hit rays are ignored
        
        # Ensure map can represent the scan boundaries
        maxDist_w = np.max(scanPolar[1,:])
        map.EnsureWorldBounds_Circle(startx_w, starty_w, maxDist_w)
        
        scanCartFilt = Utils.ScanPolarToCart(scanPolar, startDir_w)
        
        nPoints = scanCartFilt.shape[1]
        startPos = map.PosWorldToMap((startx_w, starty_w))
        
        for i in range(nPoints):
            endx_w = startx_w + scanCartFilt[0,i]
            endy_w = starty_w + scanCartFilt[1,i]
            
            endPos = map.PosWorldToMap((endx_w, endy_w))
            lineCoords = BresenhamLine.GetLine(startPos, endPos)
            
            # We know that the line has hit something
            # All cells in between are free, the final one is occupied
            
            l2IntFree = self.m_l2FreeCell
            l2IntOccupied = self.m_l2IntOccupiedCell
            
            # Iterate all cells in the line
            mapData = map.m_data
            for pos in lineCoords[0:-1]: # All except for last
                x = pos[0]
                y = pos[1]
                oldVal = mapData[x,y]
                
                newVal = oldVal - l2IntFree
                if (newVal < -128):
                    newVal = -128
                mapData[x,y] = newVal
            
            lineEnd = lineCoords[-1]
            newVal = mapData[lineEnd[0], lineEnd[1]] + l2IntOccupied
            if (newVal > 255):
                newVal = 255
            
            mapData[lineEnd[0], lineEnd[1]] = newVal
        
    def NormalizeWeights(self):
        # Normalize weights
        sumWeights = 0
        for i in range(self.m_nParticles):
            sumWeights += self.m_particles[i][3]
            
        if (sumWeights == 0):
            weight = 1 / self.m_nParticles
            for i in range(self.m_nParticles):
                self.m_particles[i][3] = weight
                return
            
        normCoeff = 1/sumWeights
        for i in range(self.m_nParticles):
            self.m_particles[i][3] = normCoeff * self.m_particles[i][3]
    
    
# END CLASS


# Interface for real map and SensorModel
class LocalizerInterfaceWorldReal(LocalizerWorldInterfaceBase):
    def __init__(self, world, sensorModel):
        self.m_world = world
        self.m_sensorModel = sensorModel
    
    # Returns xmin, ymin, xmax, ymax of world
    def GetWorldBounds(self):
        return self.m_world.GetBounds()
    
    # Returns True if given pos (x,y) is valid in the world (not inside object)
    def IsPoseValid(self, pose):
        return True
    
    # Gets full scan 
    def GetScan(self, pose, map, scanAngles):
        return self.m_sensorModel.Scan2(self.m_world, pose, scanAngles, returnNoHits = True)
    
    # Returns max range of sensor scan
    def GetScanRange(self):
        return self.m_sensorModel.GetScanRange()
    
# END CLASS

# Interface for grid map
class LocalizerInterfaceMap(LocalizerWorldInterfaceBase):
    
    def __init__(self, scanRange):
        self.m_scanRange = float(scanRange)
    
    # Returns xmin, ymin, xmax, ymax of world
    def GetWorldBounds(self):
        return (2.4, 2.4, 2.6, 2.6) # Temporary
    
    # Returns True if given pos (x,y) is valid in the world (not inside object)
    def IsPoseValid(self, pose):
        return True
    """
    This doesn't work but also it's not so important
        (x,y) = self.m_map.PosWorldToMap(self, (pose[0], pose[1]))
        value = self.m_map.GetData()[x,y]
        
        # Unknown value is ok, but not above it
        return value <= Map.Map.VALUE_UNKNOWN
    """
    
    # Returns max range of sensor scan
    def GetScanRange(self):
        return self.m_scanRange
    
    # Gets full scan 
    def GetScan(self, pose, map, scanAngles):
        (x, y, dir) = pose
        
        # Here we don't ensure we won't go out of map bounds,
        # because it makes map grow too big unnecessary

        scanOut = np.array((scanAngles, np.full(scanAngles.shape[0], -1)), dtype=float)
        scanRange = self.m_scanRange
        mapData = map.GetData_L2Int()
        
        for i in range(scanAngles.shape[0]):
            lineDir = dir + scanAngles[i]
            lineEndWorld = (x + scanRange*np.cos(lineDir),
                            y + scanRange*np.sin(lineDir))
            
            lineStartMap = map.PosWorldToMap((x,y))
            lineEndMap = map.PosWorldToMap(lineEndWorld)
            
            lineCoords = BresenhamLine.GetLine(lineStartMap, lineEndMap)
            
            
            # Iterate all points in line
            rayHitDist = float(-1)
            for mapPos in lineCoords:
                
                # Did we go out of bounds?
                if (not map.IsMapPosWithinBounds(mapPos)):
                    break;
                
                value = mapData[mapPos]
                if (value > 0):
                    # Calculate dist to this grid pos
                    hitPosWorld = map.PosMapToWorld(mapPos)
                    dx = hitPosWorld[0] - x
                    dy = hitPosWorld[1] - y
                    rayHitDist = np.sqrt(dx*dx + dy*dy)
                    break;
            
            scanOut[1][i] = rayHitDist
                
        return scanOut
    
# END CLASS