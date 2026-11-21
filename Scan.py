# -*- coding: utf-8 -*-
"""
Created on Thu Jan  1 17:01:59 2026

@author: spark
"""

import numpy as np
import Chassis
import Utils

def CompensateScanMotion(scanPolar, chassisModel, chassisInputs):
    
    steer = chassisInputs.m_steer
    moveState = chassisInputs.m_moveState
    
    if moveState == Chassis.MOVE_STATE_STOP:
        return np.copy(scanPolar)
    
    nSamples = scanPolar.shape[1]
    invMoveState = Chassis.MOVE_STATE_BACKWARD
    if moveState == Chassis.MOVE_STATE_BACKWARD:
        invMoveState = Chassis.MOVE_STATE_FORWARD
    chassisConfig = chassisModel.GetChassisConfig()
    sensorOffset_LS = (chassisConfig.m_sensorOffsetFwd,
                       chassisConfig.m_sensorOffsetSide,
                       chassisConfig.m_sensorOffsetDir_rad)
        
    scanPolarCompensated = np.zeros(scanPolar.shape, dtype=float)
    
    prevSampleDir = scanPolar[0,0]    
    dirAcc = float(0) # Accumulator, increases from 0 to 2*pi (ideally)
    tempChassisInputs = Chassis.ChassisInputs()
    for sampleId in range(nSamples):
        prevDirDiff = -Utils.WrapAngleDiff(scanPolar[0,sampleId] - prevSampleDir)
        dirAcc += prevDirDiff
        
        odoExtraParam = 1 - dirAcc / (2 * np.pi) # Extrapolation parameter 0..1
        
        # Odometer value extrapolated backwards
        extraOdo = odoExtraParam * chassisInputs.m_odoDiff
        
        # Extrapolated sample pose
        tempChassisInputs.SetValues(extraOdo, invMoveState, steer)
        
        # Where chassis origin was
        extraPoseDiff_LS = chassisModel.CalculatePoseDiff_LS(tempChassisInputs)
        
        # Convert the pose difference to Sensor Space
        extraPoseDiff_SS = chassisModel.PoseDiff_LSTOSS(extraPoseDiff_LS)
        
        extraSampleDir_SS = extraPoseDiff_SS[2] + scanPolar[0,sampleId]
        extraRayEndPos_SS = (extraPoseDiff_SS[0] + scanPolar[1,sampleId] * np.cos(extraSampleDir_SS),
                          extraPoseDiff_SS[1] + scanPolar[1,sampleId] * np.sin(extraSampleDir_SS))
        
        compRayDir = Utils.WrapAngleFull(np.atan2(extraRayEndPos_SS[1], extraRayEndPos_SS[0]))
        compRayDist = np.sqrt(extraRayEndPos_SS[0]*extraRayEndPos_SS[0] + extraRayEndPos_SS[1]*extraRayEndPos_SS[1])
        
        scanPolarCompensated[0, sampleId] = compRayDir
        scanPolarCompensated[1, sampleId] = compRayDist
        
        prevSampleDir = scanPolar[0,sampleId]
        
    return scanPolarCompensated