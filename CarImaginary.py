# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 11:48:02 2025

@author: spark
"""

import numpy as np

import CarModel
import ICP
import Utils
import Map
import Localizer
import CarStatsModel

class CarImaginary:

    def __init__(self, startPose, startOdo, carModelConfig, sensorScanRange):
        self.m_car = CarModel.CarModel(carModelConfig)
        self.m_car.SetPose(startPose)
        self.m_carConfig = carModelConfig
        
        self.m_carPose = np.array(startPose)
        
        self.m_lastOdo = np.array(startOdo)
        self.m_lastScan = np.array([])
        
        self.m_carStatsModel = CarStatsModel.CarStatsModel(posStdDevPerDist = 0.1, dirStdDevPerDist = 20*np.pi/180)
        #self.m_carStatsModel = CarStatsModel.CarStatsModel(posStdDevPerDist = 0.01, dirStdDefPerDist = 1*np.pi/180)
        localizerWorldInterface = Localizer.LocalizerInterfaceMap(sensorScanRange);
        self.m_localizer = Localizer.Localizer(startPose = startPose,
                                               nParticles = 128,
                                               sensorStdDev = float(1.5),
                                               worldInterface = localizerWorldInterface,
                                               carStatsModel = self.m_carStatsModel,
                                               mapRes_w = 0.1,
                                               mapSize_w = 20,
                                               sensorOffset = (0, 0, 0))
        
        
    def Update(self, dt, wheelOdoLR, scan, debugPlotICP):
        diffOdo = np.array(wheelOdoLR) - self.m_lastOdo # Different in ticks
        diffOdoAngle = (2*np.pi/self.m_carConfig.m_odoNTicks) * diffOdo # Difference in angles
        
        # Pose differences are relative to previous pose in local space of previous pose!
        # Forward is x increase regardless of rotation of previous pose
        
        odoPoseDiffLocal = self.m_car.CalculatePoseDiff(dt, tuple(diffOdoAngle))
        poseDiffLocal = odoPoseDiffLocal
        icpPoseDiffLocal = (float(0), float(0), float(0))
        
        
        if (self.m_lastScan.shape[0] != 0):
            print(f"ICP initial T: {odoPoseDiffLocal}")
            icpPoseDiffLocal = ICP.MatchFullPolarScans(scan, self.m_lastScan, 16, initialT = odoPoseDiffLocal, debugPlot=debugPlotICP)
            print(f"ICP final T: {icpPoseDiffLocal}")
            
            odoPoseDiffDist = np.sqrt(odoPoseDiffLocal[0]*odoPoseDiffLocal[0] + odoPoseDiffLocal[1] + odoPoseDiffLocal[1])
            icpPoseDiffDist = np.sqrt(icpPoseDiffLocal[0]*icpPoseDiffLocal[0] + icpPoseDiffLocal[1]*icpPoseDiffLocal[1])
            angleDiff = Utils.WrapAngleDiff(odoPoseDiffLocal[2] - icpPoseDiffLocal[2])
            
            odoRejectionInterval = 2.0
            
            if (icpPoseDiffDist < odoRejectionInterval*odoPoseDiffDist and
                icpPoseDiffDist > odoPoseDiffDist/odoRejectionInterval and
                np.abs(angleDiff) < np.pi/3.0):
                poseDiffLocal = icpPoseDiffLocal
            else:
                print(f"ICP result rejected! Odometer dist: {odoPoseDiffDist:.2f}, ICP dist: {icpPoseDiffDist:.2f}, Angle diff: {angleDiff:.2f}")
        
        # m_CarPose updated from odometer:
        if (False):
            self.m_carPose = Utils.AddPoseDiff(self.m_carPose, odoPoseDiffLocal)
        
        # m_CarPose updated from scan diff:
        if (False):
            self.m_carPose = Utils.AddPoseDiff(self.m_carPose, icpPoseDiffLocal)
        
        # m_CarPose updated from Localizer:
        if (True):
            # Update localizer
            self.m_localizer.Update(poseDiffLocal, scan)
            
            # Update pose from localizer
            self.m_carPose[:] = self.m_localizer.GetMostLikelyPose()
        
        self.m_lastOdo = wheelOdoLR
        self.m_lastScan = scan
        
    def GetPose(self):
        return tuple(self.m_carPose)
    
    def GetLocalizerPose(self):
        return self.m_localizer.GetMostLikelyPose()
    
# END CLASS