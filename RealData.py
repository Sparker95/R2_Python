# -*- coding: utf-8 -*-
"""
Created on Sat Aug 23 18:25:09 2025

@author: spark
"""

import numpy as np

def ScanPolarFromFile(filePath):
    fd = open(filePath, mode='r')
    
    angles = []
    distances = []
    
    # First line is not useful, it's scan number
    fd.readline()
    
    while (True):
        l = fd.readline()
        if (len(l) == 0):
            break
        
        lSplit = l.split(" ")
        angles.append(float(lSplit[0]))
        distances.append(float(lSplit[1]))
        
    scanOut = np.array([angles, distances])
    fd.close()
    return scanOut