# -*- coding: utf-8 -*-
"""
Created on Fri May  2 15:22:14 2025

@author: spark
"""

import numpy as np

import Utils

# Model of car statistics based on odometry

class CarStatsModel:
    
    def __init__(self, posStdDevPerDist, dirStdDevPerDist):
        self.m_fPosStdDevPerDist = float(posStdDevPerDist)
        self.m_fDirStdDevPerDist = float(dirStdDevPerDist)
        self.m_rng = np.random.default_rng(57895412)
        
    
    def GetPoseSample(self, poseWorld, odoPoseDiffLocal):
        (dx, dy, dDir) = odoPoseDiffLocal
        dist = np.sqrt(dx*dx + dy*dy)
        
        posStdDev = dist * self.m_fPosStdDevPerDist
        dirStdDev = dist * self.m_fDirStdDevPerDist
        
        rndx = self.m_rng.normal(0, posStdDev)
        rndy = self.m_rng.normal(0, posStdDev)
        rndDir = self.m_rng.normal(0, dirStdDev)
        
        poseDiffLocalRandomized = (odoPoseDiffLocal[0] + rndx,
                                   odoPoseDiffLocal[1] + rndy,
                                   odoPoseDiffLocal[2] + rndDir)
        
        # Rotation matrix
        mRot = Utils.MRot(poseWorld[2])
        poseDiffWorldXY = mRot@np.array((poseDiffLocalRandomized[0], poseDiffLocalRandomized[1]))
        
        return (poseWorld[0] + poseDiffWorldXY[0],
                poseWorld[1] + poseDiffWorldXY[1],
                Utils.WrapAngleFull(poseWorld[2] + poseDiffLocalRandomized[2]))
    
