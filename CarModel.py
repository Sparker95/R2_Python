# -*- coding: utf-8 -*-
"""
Created on Fri Apr 18 17:25:22 2025

@author: spark
"""

import numpy as np
import math

import Utils

class CarModelConfig:
    def __init__(self, wheelBase, wheelTrack, wheelRadius, odoNTicks):
        self.m_wheelBase = wheelBase
        self.m_wheelTrack = wheelTrack
        self.m_wheelRadius = wheelRadius
        self.m_odoNTicks = odoNTicks
                

# END CLASS

class CarModel:
    def __init__(self, carModelConfig):
        # State
        self.m_x = float(0) # Rear wheel pos
        self.m_y = float(0)
        self.m_dir = float(0)
        self.m_wheelAngleLeft = float(0)
        self.m_wheelAngleRight = float(0)
        
        # Configuration
        self.m_config = carModelConfig
        
        
    def SetPose(self, pose):
        self.m_x = pose[0]
        self.m_y = pose[1]
        self.m_dir = pose[2]
    
    # Updates state from control inputs
    def Update(self, dt, inSpeed, inSteerAngle):
        
        # Trivial case, steering is zero
        if (inSteerAngle == 0):
            path = dt*inSpeed
            self.m_x = self.m_x + path*math.cos(self.m_dir)
            self.m_y = self.m_y + path*math.sin(self.m_dir)
            diffWheelAngle = path / self.m_config.m_wheelRadius
            self.m_wheelAngleLeft += diffWheelAngle
            self.m_wheelAngleRight += diffWheelAngle
            return
        
        steerSign = np.sign(inSteerAngle) # positive -> turn right (dir decreases)
        arcRadius = abs(self.m_config.m_wheelBase/math.tan(inSteerAngle)) # For center between rear wheels
        arcPath = dt*inSpeed                   # We moved this distance along arc
        arcAngle = arcPath/arcRadius           # We rotated this angle around arc center
        arcx = self.m_x + steerSign*arcRadius*math.sin(self.m_dir) # Arc center
        arcy = self.m_y - steerSign*arcRadius*math.cos(self.m_dir)
        
        self.m_dir = self.m_dir - steerSign*arcAngle               # Turn left -> dir increases
        self.m_dir = Utils.WrapAngleFull(self.m_dir)
        self.m_x = arcx - steerSign*arcRadius*math.sin(self.m_dir)
        self.m_y = arcy + steerSign*arcRadius*math.cos(self.m_dir)
        
        arcRadiusIn = arcRadius - 0.5*self.m_config.m_wheelTrack  # Inner
        arcRadiusOut = arcRadius + 0.5*self.m_config.m_wheelTrack # Outer
        arcPathIn = arcAngle*arcRadiusIn
        arcPathOut = arcAngle*arcRadiusOut
        diffWheelAngleIn = arcPathIn/self.m_config.m_wheelRadius
        diffWheelAngleOut = arcPathOut/self.m_config.m_wheelRadius
        
        if (steerSign > 0):
            self.m_wheelAngleLeft += diffWheelAngleOut # Turn right -> left is outer wheel
            self.m_wheelAngleRight += diffWheelAngleIn
        else:
            self.m_wheelAngleLeft += diffWheelAngleIn # Turn left -> left is inner wheel
            self.m_wheelAngleRight += diffWheelAngleOut
    
    # Calculates pose diff based on odometer diff
    # Returns (diffx, diffy, diffDir)
    def CalculatePoseDiff(self, dt, diffWheelAngleLR):
        (diffWheelAngleLeft, diffWheelAngleRight) = diffWheelAngleLR
        
        #self.m_wheelAngleLeft += diffWheelAngleLeft
        #self.m_wheelAngleRight += diffWheelAngleRight
        
        # Trivial case, same odometers, zero steering
        if (diffWheelAngleLeft == diffWheelAngleRight):
            path = diffWheelAngleLeft * self.m_config.m_wheelRadius
            diffx = path
            diffy = 0
            return (diffx, diffy, float(0.0))
        
        steerSign = 1 # positive -> turn right (dir decreases)
        if (diffWheelAngleLeft < diffWheelAngleRight):
            steerSign = -1
        
        diffWheelAngleIn = float(0)
        diffWheelAngleOut = float(0)
        
        if (steerSign > 0):
           diffWheelAngleIn = diffWheelAngleRight
           diffWheelAngleOut = diffWheelAngleLeft
        else:
           diffWheelAngleIn = diffWheelAngleLeft
           diffWheelAngleOut = diffWheelAngleRight
        
        #arcPathIn = diffWheelAngleIn * self.m_config.m_wheelRadius
        #arcPathOut = diffWheelAngleOut * self.m_config.m_wheelRadius
        arcRadiusIn = diffWheelAngleIn*self.m_config.m_wheelTrack / (diffWheelAngleOut-diffWheelAngleIn)
        arcAngle = self.m_config.m_wheelRadius*(diffWheelAngleOut-diffWheelAngleIn) / self.m_config.m_wheelTrack
        
        arcRadius = arcRadiusIn + 0.5*self.m_config.m_wheelTrack # Centerpoint between wheels
        #arcx = self.m_x + steerSign*arcRadius*math.sin(self.m_dir) # Arc center
        #arcy = self.m_y - steerSign*arcRadius*math.cos(self.m_dir)
        
        # Calculate differences of x,y,dir
        # It is rewritten this way so that we can return pose diffs out of Update call
        diffDir = -steerSign*arcAngle # Turn left -> dir increases
        diffx = steerSign*arcRadius*(-math.sin(diffDir))	
        diffy = steerSign*arcRadius*(-1 + math.cos(diffDir))
        
        return (diffx, diffy, diffDir)
        
    
    #----------------------------------------------------------------
    # Getters
    
    def GetPose(self):
        return (self.m_x, self.m_y, self.m_dir)
    
    def GetWheelAngles(self):
        return (self.m_wheelAngleLeft, self.m_wheelAngleRight)
    
    def GetOdometers(self):
        tickAngle = 2*np.pi/self.m_config.m_odoNTicks
        l = np.floor(self.m_wheelAngleLeft/tickAngle)
        r = np.floor(self.m_wheelAngleRight/tickAngle)
        return (l, r)
    
# END CLASS
