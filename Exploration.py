# -*- coding: utf-8 -*-
"""
Created on Sun Jun 28 21:34:10 2026

@author: spark
"""

import collections

import numpy as np
import matplotlib.pyplot as plt

import Map
import PathFinding

def CalculateExplorationMap(map, distMap, startPos_w, distMapThreshold_w):
    mapData = map.GetData_L2Int()
    startCell = map.PosWorldToMap(startPos_w)
    
    distMapThreshold_m = PathFinding.WorldToDistanceMapValue(distMapThreshold_w, map.GetResolution())
    
    expMap = -1 * np.ones(mapData.shape, dtype = int)
    
    expMap[startCell] = 0
    frontCells = collections.deque()
    frontCells.append(startCell)
    
    expCells = set() # Final set of cells for exploration

    neighborOffsetAndCost = ((0,  1, 3),
                          (1,  1, 4),
                          (1,  0, 3),
                          (1, -1, 4),
                          (0, -1, 3),
                          (-1,-1, 4),
                          (-1, 0, 3),
                          (-1, 1, 4))
    
    while len(frontCells) > 0:
        thisCell = frontCells.popleft()
        thisValue = expMap[thisCell]
        for offAndCost in neighborOffsetAndCost:
            neighborCell = (thisCell[0] + offAndCost[0],
                            thisCell[1] + offAndCost[1])
            
            # Ignore if out of bounds
            if not map.IsMapPosWithinBounds(neighborCell):
                continue
            
            # Ignore if occupied
            if (mapData[neighborCell] > 0):
                continue;
            
            # Ignore if can't fit there
            if (distMap[neighborCell] <= distMapThreshold_m):
                continue                
            
            newNeighborValue = thisValue + offAndCost[2]
            neighborValue = expMap[neighborCell]
            
            if (neighborValue == -1):
                # Never visited this cell
                expMap[neighborCell] = newNeighborValue
                
                # Add it to front, or to cells for exploration?
                if (mapData[neighborCell] == 0):
                    # Cell occupancy is unknown - add it to cells for exploration
                    if (not neighborCell in expCells):
                        expCells.add(neighborCell)
                else:
                    # Regular most likely free cell - add it to front
                    if (not neighborCell in frontCells):
                        frontCells.append(neighborCell)
            elif (newNeighborValue < neighborValue):
                # Already visited but more expensive
                expMap[neighborCell] = newNeighborValue
    
    return (expCells, expMap)

def GetCellNeighbors(cell, cellsSet):
    neighborCells = []
    offsets = ((0,  1),
               (1,  1),
               (1,  0),
               (1, -1),
               (0, -1),
               (-1,-1),
               (-1, 0),
               (-1, 1))
    for offset in offsets:
        neighborCell = (cell[0] + offset[0], cell[1] + offset[1])
        if neighborCell in cellsSet:
            neighborCells.append(neighborCell)
    
    return neighborCells

# Finds cells in cellsSet which are connected to the cell
def FindConnectedCells(cell, cellsSet):
    connectedCells = set()
    connectedCells.add(cell)
    
    que = collections.deque()
    que.append(cell)
    
    while (len(que) > 0):
        thisCell = que.popleft()
        
        neighborCells = GetCellNeighbors(thisCell, cellsSet)
        for neighborCell in neighborCells:
            if not neighborCell in connectedCells:
                que.append(neighborCell) # If array is used, remember to check if cell is not added!
                connectedCells.add(neighborCell)
            
    return connectedCells

# Returns all graph components in the cellsSet
# Returns list of sets
def FindCellGraphComponents(cellsSet):    
    cellsSetCopy = cellsSet.copy() # Working copy of the set, we will modify it
    componentsList = []
    
    while len(cellsSetCopy) > 0:
        anyCell = next(iter(cellsSetCopy))
        anyCellComponent = FindConnectedCells(anyCell, cellsSetCopy) # Means: component of the anyCell
        componentsList.append(anyCellComponent)
        cellsSetCopy -= anyCellComponent # Update the set, removing elements found in others.
    
    return componentsList

# Returns score and nearest cell: (score, nearestCell)
def EvaluateCellComponent(cellsSet, explorationMap):
    # Closer is better, bigger number of connected cells is better
    # distScore = 1 / (1 + min dist to any cell in cellsSet)
    # compSizeScore = cellsSet.size()
    # score = compSizeScore * distScore
    
    nearestCell = next(iter(cellsSet))
    minDist = explorationMap[nearestCell]
    
    for cell in cellsSet:
        dist = explorationMap[cell]
        if (dist < minDist):
            minDist = dist
            nearestCell = cell
    
    distScore = 1 / (1 + minDist)
    compSizeScore = len(cellsSet)
    score = 1000 * distScore * compSizeScore # Scale for better range
    
    return (score, nearestCell)
