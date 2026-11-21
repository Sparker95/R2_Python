# -*- coding: utf-8 -*-
"""
Created on Sat Apr 19 12:36:06 2025

@author: spark
"""

import numpy as np

class SensorModel:
    def __init__(self, scanRange, nPointsInScan):
        self.m_scanRange = float(scanRange)
        self.m_nPointsInScan = nPointsInScan # Samples in each scan
        
    # Performs circular scan.
    # Angles are taken uniformly.
    # Returns: array of distances. -1 if didn't hit anything.
    def Scan(self, world, pose):
        (x, y, dir) = pose
        nPoints = self.m_nPointsInScan
        scanRange = self.m_scanRange
        angles = []
        distances = []
        
        for i in range(nPoints):
            lineDirRel = i*2*np.pi/nPoints # Relative to pose
            lineDir = dir + lineDirRel
            line = (x, y,
                    x + scanRange*np.cos(lineDir),
                    y + scanRange*np.sin(lineDir))
            (hit, xhit, yhit, thit) = world.TraceLine(line)
            
            # Only hits are appended
            if (hit):
                hitDist = thit*scanRange
                angles.append(lineDirRel)
                distances.append(hitDist)
                
        return np.array((angles, distances), dtype=float)
    
    # Same as Scan(), but receives array of directions relative to pose direciton
    # returnNoHits: when True, returns EXACTLY same angles as passed to it, even in case of no-hits!
    def Scan2(self, world, pose, inAngles, returnNoHits = True):
        (x, y, dir) = pose
        scanRange = self.m_scanRange
        angles = []    # Arrays with output data
        distances = [] #
        
        for angleRel in inAngles:
            lineDir = dir + angleRel
            line = (x, y,
                    x + scanRange*np.cos(lineDir),
                    y + scanRange*np.sin(lineDir))
            (hit, xhit, yhit, thit) = world.TraceLine(line)
            
            # Only hits are appended
            if (hit):
                hitDist = thit*scanRange
                angles.append(lineDir)
                distances.append(hitDist)
            elif (returnNoHits):
                angles.append(lineDir)
                distances.append(float(-1))
                
                
        return np.array((angles, distances), dtype=float)
    
    def GetScanRange(self):
        return self.m_scanRange
    
# END CLASS
