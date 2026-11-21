# -*- coding: utf-8 -*-
"""
Created on Sat Nov 29 15:11:15 2025

@author: spark
"""

# Handling of various data from chassis, like odometer, steering, etc

import Utils

import math
import numpy as np

MOVE_STATE_STOP = int(0)
MOVE_STATE_FORWARD = int(1)
MOVE_STATE_BACKWARD = int(2)

def CalculateOdometerDifference(odoNew, odoOld):
    odoNew = int(odoNew)
    odoOld = int(odoOld)
    return (odoNew - odoOld) & 0xFFFF 

class ChassisConfig:
    def __init__(self):
        # Wheels
        self.m_wheelBase   = 0.26
        self.m_trackWidth  = 0.17
        self.m_wheelRadius = 0.03125
        self.m_width       = 0.22
        self.m_length      = 0.38
        
        # Radians per unit of input, see ChassisMeasurement
        self.m_steerSensitivity_rad = (math.pi / 180) * (23 / 0.68)
        
        
        # Sensor
        self.m_sensorOffsetFwd     = 0.185 # Relative to rear axle center point
        self.m_sensorOffsetSide    = float(0)
        self.m_sensorOffsetDir_rad = 0.5 * math.pi # Counter clock wise
        
        # Transmission
        # gear Motor -> gear Shaft -> (gear Diff In -> gear Diff Out) -> wheel Radius
        #                    |
        #                    V
        #               gear Encoder -> encoder Pulses
        
        self.m_gearMotor   = 14 # Not used
        self.m_gearShaft   = 72 # Not used
        self.m_gearDiffIn  = 13
        self.m_gearDiffOut = 38
        
        # Encoder - mounted on main shaft
        self.m_encoderPulses = 20 # Per revolution

# Converts pose difference from Local Space to Sensor Space
def PoseDiff_LSTOSS(chassisConfig, poseDiff_LS):
    (dx, dy, ddir) = poseDiff_LS
    mRot = Utils.MRot(-chassisConfig.m_sensorOffsetDir_rad)
    poseDiff_XY_SS = mRot@np.array((dx, dy))
    return (poseDiff_XY_SS[0], poseDiff_XY_SS[1], ddir)

# Converts pose difference from Sensor Space to Local Space
def PoseDiff_SSTOLS(chassisConfig, poseDiff_SS):
    (dx, dy, ddir) = poseDiff_SS
    mRot = Utils.MRot(chassisConfig.m_sensorOffsetDir_rad)
    poseDiff_XY_LS = mRot@np.array((dx, dy))
    return (poseDiff_XY_LS[0], poseDiff_XY_LS[1], ddir)
    
class ChassisStatsModelConfig:
    def __init__(self):
        self.m_wheelRadiusTol_abs =   float(0.003)          # +- Meters
        self.m_odoFreeplay_abs =      float(1)              # +- Number of pulses
        self.m_wheelFreeplay_rad =    float(20 * np.pi / 180) # +- Radians
        self.m_steerTolStraight_abs = float(0.25)            # +- Steering units
        self.m_steerTolTurn_abs =     float(0.5)            # +- Steering units
    
class ChassisInputs:
    def __init__(self):
        self.m_odoDiff = int(0)
        self.m_moveState = MOVE_STATE_STOP
        self.m_steer = float(0)
    
    def SetValues(self, odoDiff, moveState, steer):
        self.m_odoDiff = odoDiff
        self.m_moveState = moveState
        self.m_steer = steer
        
    
class ChassisModel:
    def __init__(self, chassisConfig, statsConfig):
        self.m_chassisConfig = chassisConfig
        self.m_statsConfig = statsConfig
        self.m_rng = np.random.default_rng(9784)
    
    def GetChassisConfig(self):
        return self.m_chassisConfig
    
    def GetStatsConfig(self):
        return self.m_statsConfig
    
    # Calculates ideal pose difference, without randomization
    # Returns (diffx, diffy, diffDir), in local space
    def CalculatePoseDiff_LS(self, chassisInputs):
        odoDiff = chassisInputs.m_odoDiff
        moveState = chassisInputs.m_moveState
        steerInput = chassisInputs.m_steer
        
        # Stopped:
        if (moveState == MOVE_STATE_STOP):
            return (float(0), float(0), float(0))
        
        c = self.m_chassisConfig
        distPerOdoPulse = ( (1 / c.m_encoderPulses) * (c.m_gearDiffIn / c.m_gearDiffOut) * 2 * math.pi * c.m_wheelRadius )
        moveDistAbs = float(odoDiff) * distPerOdoPulse
        
        moveSign = float(1)
        steerSign = float(1)
        if (moveState == MOVE_STATE_BACKWARD):
            moveSign = float(-1)
        if (steerInput < 0):
            steerSign = float(-1)
        
        # Moving straight:
        if (steerInput == 0):
            return (moveSign * moveDistAbs, 0, 0)
        
        # Turn radius of rear axle center point
        innerWheelAngle_rad = abs(steerInput) * c.m_steerSensitivity_rad
        innerWheelTurnRadius = c.m_wheelBase / math.tan(innerWheelAngle_rad)
        rearAxleCenterTurnRadius = 0.5 * c.m_trackWidth + innerWheelTurnRadius
        
        moveArcAngle = moveDistAbs / rearAxleCenterTurnRadius
        
        ddir = - steerSign * moveSign * moveArcAngle # Invesrsed because positive steering rotates clockwise
        dx =  moveSign * rearAxleCenterTurnRadius * math.sin(moveArcAngle) # Forward
        dy = - steerSign * abs(dx) * math.tan(moveArcAngle)
        
        return (dx, dy, ddir)
    
    
    # Calculates randomized pose difference
    # outInputsRandomized - the randomized inputs values which were chosen for calculation
    def CalculateRandomPoseDiff_LS(self, chassisInputs, outInputsRandomized):
        odoDiff = chassisInputs.m_odoDiff
        moveState = chassisInputs.m_moveState
        steerInput = chassisInputs.m_steer
        
        # Stopped:
        if (moveState == MOVE_STATE_STOP):
            outInputsRandomized.SetValues(odoDiff, moveState, steerInput)
            return (float(0), float(0), float(0))
        
        chassisConf = self.m_chassisConfig
        statsConf = self.m_statsConfig
        
        # Calculate and randomize travel distance
        wheelRadius_rnd = self.m_rng.uniform(chassisConf.m_wheelRadius - statsConf.m_wheelRadiusTol_abs, chassisConf.m_wheelRadius + statsConf.m_wheelRadiusTol_abs)
        
        wheelTurnAnglePerOdoPulse_rad = ( (1 / chassisConf.m_encoderPulses) * (chassisConf.m_gearDiffIn / chassisConf.m_gearDiffOut) * 2 * math.pi)
        
        odoDiff_rnd = self.m_rng.uniform(odoDiff - statsConf.m_odoFreeplay_abs, odoDiff + statsConf.m_odoFreeplay_abs)
        
        wheelTurnAngle_rad = wheelTurnAnglePerOdoPulse_rad * odoDiff_rnd + self.m_rng.uniform(-statsConf.m_wheelFreeplay_rad, statsConf.m_wheelFreeplay_rad)
        
        moveDist_abs = wheelTurnAngle_rad * wheelRadius_rnd
        
        # Randomize steer input
        steerTol_abs = statsConf.m_steerTolTurn_abs
        if steerInput == 0:
            steerTol_abs = statsConf.m_steerTolStraight_abs                
        steerInput_rnd = self.m_rng.uniform(steerInput - steerTol_abs, steerInput + steerTol_abs)
        
        moveSign = float(1)
        steerSign = float(1)
        if (moveState == MOVE_STATE_BACKWARD):
            moveSign = float(-1)
        if (steerInput_rnd < 0):
            steerSign = float(-1)
            
        # Return the randomized inputs
        outInputsRandomized.SetValues(odoDiff_rnd, moveState, steerInput_rnd)
        
        # Moving straight:
        if (steerInput_rnd == 0):
            return (moveSign * moveDist_abs, 0, 0)
        
        # Turn radius of rear axle center point
        innerWheelAngle_rad = abs(steerInput_rnd) * chassisConf.m_steerSensitivity_rad
        innerWheelTurnRadius = chassisConf.m_wheelBase / math.tan(innerWheelAngle_rad)
        rearAxleCenterTurnRadius = 0.5 * chassisConf.m_trackWidth + innerWheelTurnRadius
        
        moveArcAngle = moveDist_abs / rearAxleCenterTurnRadius
        
        ddir = - steerSign * moveSign * moveArcAngle # Invesrsed because positive steering rotates clockwise
        dx =  moveSign * rearAxleCenterTurnRadius * math.sin(moveArcAngle) # Forward
        dy = - steerSign * abs(dx) * math.tan(moveArcAngle)
        
        return (dx, dy, ddir)
    
    # Converts pose difference from Local Space to Sensor Space
    def PoseDiff_LSTOSS(self, poseDiff_LS):
        (dx, dy, ddir) = poseDiff_LS
        chassisConfig = self.m_chassisConfig
        mRot = Utils.MRot(-chassisConfig.m_sensorOffsetDir_rad)
        poseDiff_XY_SS = mRot@np.array((dx, dy))
        return (poseDiff_XY_SS[0], poseDiff_XY_SS[1], ddir)

    # Converts pose difference from Sensor Space to Local Space
    def PoseDiff_SSTOLS(self, poseDiff_SS):
        (dx, dy, ddir) = poseDiff_SS
        chassisConfig = self.m_chassisConfig
        mRot = Utils.MRot(chassisConfig.m_sensorOffsetDir_rad)
        poseDiff_XY_LS = mRot@np.array((dx, dy))
        return (poseDiff_XY_LS[0], poseDiff_XY_LS[1], ddir)