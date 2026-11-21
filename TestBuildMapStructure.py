# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 22:19:19 2026

@author: spark
"""

import PathFinding

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

import Map
import MapUtils
import Utils

map = MapUtils.ReadMapFromFile(r"D:\Projects\Robot2\Records\Apartment.rmap")
distMap = PathFinding.CalculateDistanceMap(map.m_data)
structureMap = np.zeros(distMap.shape, dtype=int)

fig, ax = plt.subplots()
fig.suptitle("Original map")
Utils.PlotMap(map, ax, "Greys")

####
offsets = ((-1,1),  (0,1),  (1,1),
           (-1,0),  (0,0),  (1,0), # Center offset is not needed
           (-1,-1), (0,-1), (1,-1))
offsetPairs = (((-1,0), (1,0)),
               ((0,-1), (0,1)),
               ((-1,-1), (1,1)),
               ((-1,1), (1,-1)))


# for x in range(1, distMap.shape[0]-1):
#     for y in range(1, distMap.shape[1]-1):
#         centerValue = distMap[x,y]
#         maxValue = centerValue
#         for offset in offsets:
#             valueAtOffset = distMap[x+offset[0], y+offset[1]]
#             if valueAtOffset > maxValue:
#                 maxValue = valueAtOffset
        
#         if maxValue == centerValue and maxValue != 0:
#             structureMap[x,y] = 1
#         else:
#             structureMap[x,y] = 0

for x in range(1, distMap.shape[0]-1):
    for y in range(1, distMap.shape[1]-1):
        centerValue = distMap[x,y]
        
        ridge = False
        for offsetPair in offsetPairs:
            v0 = distMap[x+offsetPair[0][0], y+offsetPair[0][1]]
            v1 = distMap[x+offsetPair[1][0], y+offsetPair[1][1]]
            
            if centerValue > v0 and centerValue > v1:
                ridge = True
                break
            
        if ridge and centerValue != 0:
            structureMap[x,y] = 1
        else:
            structureMap[x,y] = 0
            
fig, ax = plt.subplots()
fig.suptitle("Structure map")
#ax.imshow(structureMap)
Utils.PlotDistMap(map, structureMap, ax)

# Make structure wider

structureDistMap = PathFinding.CalculateDistanceMap(structureMap, treshold = 1)

fig, ax = plt.subplots()
fig.suptitle("Structure Distance Map")
ax.imshow(structureDistMap)

structureWidth_w = 0.15
minDistToObstacle_w = 0.1

structureDistMapTreshold = structureWidth_w / map.m_res_w * 3
distToObstacleThreshold = minDistToObstacle_w / map.m_res_w * 3

for x in range(0, structureDistMap.shape[0]):
    for y in range(0, structureDistMap.shape[1]):
        if (structureDistMap[x,y] < structureDistMapTreshold and # Close to structure
            distMap[x,y] > distToObstacleThreshold): # Not too close to obstacle - physically never can be there
            structureDistMap[x,y] = 1
        else:
            structureDistMap[x,y] = 0
            
fig, ax = plt.subplots()
fig.suptitle("Map Skeleton (Widened)")
#ax.imshow(structureDistMap)
Utils.PlotStructMap(map, structureDistMap, ax)           
            
            