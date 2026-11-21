# -*- coding: utf-8 -*-
"""
Created on Thu Apr 24 23:27:37 2025

@author: spark
"""

import numpy as np

import Utils

class Map:

    def __init__(self, xSizeCells, ySizeCells, resolution_w, allocData = True):
        self.m_res_w = float(resolution_w)
        self.m_xSize = xSizeCells
        self.m_ySize = ySizeCells
        self.m_xCenter = int(self.m_xSize / 2)
        self.m_yCenter = int(self.m_ySize / 2)
        if allocData:
            self.m_data = np.zeros((self.m_xSize, self.m_ySize), dtype=int)
        else:
            self.m_data = np.zeros((2,2), dtype = int)
        
    # Deep copy
    def Copy(self):
        c = Map(self.m_xSize, self.m_ySize, self.m_res_w, allocData = False)
        c.m_xCenter = self.m_xCenter
        c.m_yCenter = self.m_yCenter
        c.m_data = np.copy(self.m_data)
        return c
    
    def SetCenter(self, xCenter, yCenter):
        self.m_xCenter = xCenter
        self.m_yCenter = yCenter
    
    # Returns data in L2 integer format
    def GetData_L2Int(self):
        return self.m_data
    
    # Returns data in probability format
    def GetData_Prob(self):
        probabilities = np.zeros((self.m_xSize, self.m_ySize), dtype=float)
        
        for x in range(self.m_xSize):
            for y in range(self.m_ySize):
                l2Int = self.m_data[x,y]
                l2Float = Utils.L2IntToL2Float(l2Int)
                prob = Utils.L2FloatToProb(l2Float)
                probabilities[x,y] = prob
        return probabilities
    
    # Returns cell size
    def GetResolution(self):
        return self.m_res_w
    
    def PosWorldToMap(self, posWorld):
        (xWorld, yWorld) = posWorld
        
        return (self.m_xCenter + int(np.floor(xWorld / self.m_res_w)),
                self.m_yCenter + int(np.floor(yWorld / self.m_res_w)))
    
    # Center of given grid cell
    def PosMapToWorld(self, posMap):
        (xMap, yMap) = posMap
        
        dxMap = xMap - self.m_xCenter # Relative to map center
        dyMap = yMap - self.m_yCenter
        
        halfres = 0.5*self.m_res_w
        return (dxMap*self.m_res_w + halfres, dyMap*self.m_res_w + halfres)
    
    def PosWorldUnscaledToMap(self, posWorldUnscaled):
        (xWorld_u, yWorld_u) = posWorldUnscaled
        
        return (self.m_xCenter + int(np.floor(xWorld_u)),
                self.m_yCenter + int(np.floor(yWorld_u)))
    
    def PosMapToWorldUnscaled(self, posMap):
        (xMap, yMap) = posMap
        
        dxMap = xMap - self.m_xCenter # Relative to map center
        dyMap = yMap - self.m_yCenter
        
        return (dxMap + 0.5, dyMap + 0.5)
    
    def IsMapPosWithinBounds(self, posMap):
        return posMap[0] >= 0 and posMap[0] < self.m_xSize and posMap[1] >= 0 and posMap[1] < self.m_ySize

    # Returns world extents (xmin_w, ymin_w, xmax_w, ymax_w)
    def GetWorldBounds(self):
        xmax_w = self.m_res_w * (self.m_xSize - self.m_xCenter)
        xmin_w = -self.m_res_w * self.m_xCenter
        ymax_w = self.m_res_w* (self.m_ySize - self.m_yCenter)
        ymin_w = -self.m_res_w * self.m_yCenter
        return (xmin_w, ymin_w, xmax_w, ymax_w)

    # Ensures that given rectangle (in world coordinates) can be represented within map
    # Resizes the map accordingly
    def EnsureWorldBounds_Rect(self, xmin_w, ymin_w, xmax_w, ymax_w):
        (xmin, ymin) = self.PosWorldToMap((xmin_w, ymin_w))
        (xmax, ymax) = self.PosWorldToMap((xmax_w, ymax_w))
        
        xmin = min(xmin, 0)
        xmax = max(xmax, self.m_xSize)
        ymin = min(ymin, 0)
        ymax = max(ymax, self.m_ySize)
        
        extendRight = xmax > self.m_xSize
        extendLeft = xmin < 0
        extendUp = ymax > self.m_ySize
        extendDown = ymin < 0
                
        if (not extendRight and not extendLeft and not extendUp and not extendDown):
            return # No need to increase size
        
        
        # Calculate new size, find new center position.
        newxSize = self.m_xSize
        newySize = self.m_ySize
        newxCenter = self.m_xCenter
        newyCenter = self.m_yCenter
        
        if (extendLeft or extendRight):
            newxSize = 128 + self.m_xSize # Ensure size is always increased
            while (newxSize < (xmax - xmin + 1)):
                newxSize = 128 + newxSize
            
            newxCenter = self.m_xCenter
                
            if (extendLeft and extendRight):
                newxCenterInNewBounds = self.m_xCenter - xmin # xmin is negative
                newxCenter = newxCenterInNewBounds + int((newxSize - (xmax - xmin)) >> 1) # Equally extend both ways
            elif (extendLeft):
                newxCenter = newxSize - (self.m_xSize - self.m_xCenter)
                
        if (extendDown or extendUp):
            newySize = 128 + self.m_ySize # Ensure size is always increased
            while (newySize < (ymax - ymin + 1)):
                newySize = 128 + newySize
            
            newyCenter = self.m_yCenter
                
            if (extendDown and extendUp):
                newyCenterInNewBounds = self.m_yCenter - ymin # xmin is negative
                newyCenter = newyCenterInNewBounds + int((newySize - (ymax - ymin)) >> 1) # Equally extend both ways
            elif (extendDown):
                newyCenter = newySize - (self.m_ySize - self.m_yCenter)
                    
        # Copy data
        newData = np.zeros((newxSize, newySize), dtype=int)
        xCopyStart = newxCenter-self.m_xCenter
        xCopyEnd = newxCenter + self.m_xSize - self.m_xCenter
        yCopyStart = newyCenter - self.m_yCenter
        yCopyEnd = newyCenter + self.m_ySize - self.m_yCenter
        newData[xCopyStart : xCopyEnd, yCopyStart : yCopyEnd] = self.m_data
        
        # Update member variables
        self.m_data = newData
        self.m_xCenter = newxCenter
        self.m_yCenter = newyCenter
        self.m_xSize = newxSize
        self.m_ySize = newySize
        
    def EnsureWorldBounds_Circle(self, x_w, y_w, radius_w):
        self.EnsureWorldBounds_Rect(x_w - radius_w, y_w - radius_w, x_w + radius_w, y_w + radius_w)
        
    # Shrinks map to its contents
    def Shrink(self):        
        xmin = self.m_xSize
        xmax = 0
        ymin = self.m_ySize
        ymax = 0
        
        for y in range(0, self.m_ySize):
            for x in range(0, self.m_xSize):
                if (self.m_data[x,y] != 0):
                    if (x < xmin):
                        xmin = x
                    if (x > xmax):
                        xmax = x
                    if (y < ymin):
                        ymin = y
                    if (y > ymax):
                        ymax = y
                        
        if (xmax < xmin and ymax < ymin):
            return
        
        newxSize = xmax - xmin + 1
        newySize = ymax - ymin + 1
        
        newData = np.zeros((newxSize, newySize), dtype=int)
        
        for y in range(0, newySize):
            for x in range(0, newxSize):
                newData[x,y] = self.m_data[xmin+x, ymin+y]
        
        self.m_xSize = newxSize
        self.m_ySize = newySize
        self.m_data = newData
        self.m_xCenter = self.m_xCenter - xmin
        self.m_yCenter = self.m_yCenter - ymin

# END CLASS