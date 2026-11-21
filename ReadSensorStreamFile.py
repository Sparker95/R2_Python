# -*- coding: utf-8 -*-
"""
Created on Sat Nov 29 16:25:08 2025

@author: spark
"""

import SensorStreamFile
import Chassis
import Utils
import ICP
import Localizer
import CarStatsModel
import Scan

import numpy as np
import matplotlib.pyplot as plt

streamBlocks = SensorStreamFile.ReadSensorStream(r"D:\Projects\Robot2\Records\Circle.rstream")

chassisConfig = Chassis.ChassisConfig()
chassisStatsConfig = Chassis.ChassisStatsModelConfig()

aOdoPoseX = []
aOdoPoseY = []
aSensorPoseX = []
aSensorPoseY = []
poseBlockId = [] # Index of blocks appended to poseX, poseY
pose_array = []

aLocalizerPoseX = []
aLocalizerPoseY = []

aOdoPoseDiff_LS = []
aSensorPoseDiff_LS = []

# Other objects
startPose = (float(0), float(0), float(0))
#carStatsModel = CarStatsModel.CarStatsModel(posStdDevPerDist = 0.2, dirStdDevPerDist = 40*np.pi/180)
#carStatsModel = CarStatsModel.CarStatsModel(posStdDevPerDist = 1e-5, dirStdDevPerDist = 1e-5)

chassisModel = Chassis.ChassisModel(chassisConfig, chassisStatsConfig)
localizerWorldInterface = Localizer.LocalizerInterfaceMap(scanRange = float(12))
localizer = Localizer.Localizer(startPose = startPose,
                                nParticles = 64,
                                sensorStdDev = float(2 * 0.0625),
                                sensorPdfFloor = float(0.6),
                                worldInterface = localizerWorldInterface,
                                chassisStatsModel = chassisModel,
                                mapRes_w = 0.0625,
                                sensorOffset = (chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad),
                                l2OccupiedCell = float(3), l2FreeCell = float(0.5))


sensorPose = (0, 0, 0)
odoPose = (0, 0, 0)
prevBlock = streamBlocks[0]
chassisInputs = Chassis.ChassisInputs()
for blockId in range(len(streamBlocks)): #range(21):
    print(f"Processing block {blockId} / {len(streamBlocks)}")
    block = streamBlocks[blockId]
    odoDiff = Chassis.CalculateOdometerDifference(block.m_Odometer, prevBlock.m_Odometer)
    
    if (odoDiff == 0):    
        prevBlock = block
        continue
    
    chassisInputs.SetValues(odoDiff, block.m_MoveState, block.m_Steer)
    
    odoPoseDiff_LS = chassisModel.CalculatePoseDiff_LS(chassisInputs)
    odoPoseDiff_SS = chassisModel.PoseDiff_LSTOSS(odoPoseDiff_LS) # Odometer pose diff in Sensor Space
    
    odoPose = Utils.AddPoseDiff(odoPose, odoPoseDiff_LS)
    aOdoPoseX.append(odoPose[0])
    aOdoPoseY.append(odoPose[1])
    
    poseBlockId.append(blockId)
    
    # Localizer
    if (True):
        #localizer.Update(sensorPoseDiff_LS, block.m_ScanPolar)
        localizer.Update(chassisInputs, block.m_ScanPolar) # !!! Localizer uses odometer, not sensor's pose diff!
        locPose = localizer.GetMostLikelyPose()
        aLocalizerPoseX.append(locPose[0])
        aLocalizerPoseY.append(locPose[1])
    
    prevBlock = block

# Plot sensor path
if (False):
    fig, ax = plt.subplots()
    ax.grid(True)
    ax.plot(aSensorPoseX, aSensorPoseY, '.-b')
    fig.suptitle("Sensor  path")
    
    for i in range(len(aSensorPoseX)):
        blockid = poseBlockId[i]
        ax.annotate(f"{blockid}", xy=(aSensorPoseX[i], aSensorPoseY[i]))

# Plot odometer path
if (True):
    fig, ax = plt.subplots()
    ax.grid(True)
    ax.plot(aOdoPoseX, aOdoPoseY, '.-b')
    fig.suptitle("Odometer  path")
    
    for i in range(len(aOdoPoseX)):
        blockid = poseBlockId[i]
        ax.annotate(f"{blockid}", xy=(aOdoPoseX[i], aOdoPoseY[i]))

# Plot localizer path
if (True):
    fig, ax = plt.subplots()
    ax.grid(True)
    ax.plot(aLocalizerPoseX, aLocalizerPoseY, '.-r')
    fig.suptitle("Localizer path")

# Plot localizer map
if (True):
    mapFig, mapAx = plt.subplots()
    bestParticle = localizer.GetMostLikelyParticle()
    Utils.PlotMap(bestParticle[4], mapAx)

# Plot lozalizer history
if (False):
    for i in range(0, len(localizer.m_particlesHistory), 8):
        locFig, locAx = plt.subplots()
        locFig.suptitle(f"Localizer {i}")
        Utils.PlotLocalizer(localizer.m_particlesHistory[i], locAx)

# Plot the path of most likely particle
if (True):
    fig, ax = plt.subplots()
    fig.suptitle("Path of most likely particle")
    particle = localizer.GetMostLikelyParticle()
    partHistory = particle[5]
    partHistoryX = np.zeros(len(partHistory))
    partHistoryY = np.zeros(len(partHistory))
    for i in range(len(partHistory)):
        partHistoryX[i] = partHistory[i][0]
        partHistoryY[i] = partHistory[i][1]
    ax.plot(partHistoryX, partHistoryY, '.-b')
    ax.grid(True)

plt.show()
