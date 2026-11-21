# -*- coding: utf-8 -*-
"""
Created on Sun Jun 28 18:37:12 2026

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt

import Map
import MapUtils
import Utils
import PathFinding
import Exploration

#MAP_PATH = r"D:\Projects\Robot2\Records\Office 17 06 2026\Office2.rmap"
#START_POS_W = (4, -2)

#MAP_PATH = r"D:\Projects\Robot2\Records\Office 17 06 2026\Office3.rmap"
#START_POS_W = (2, 0)

#MAP_PATH = "D:\Projects\Robot2\Records\Legacy Encoder\Office 26 Jan 2026\Office1.rmap"
#START_POS_W = (0, -13)

MAP_PATH = r"D:\Projects\Robot2\Records\Apartment 28 06 2026\Room2.rmap"
START_POS_W = (1.6, -0.2)

map = MapUtils.ReadMapFromFile(MAP_PATH)

# Plot original map
fig, ax = plt.subplots()
Utils.PlotMap(map, ax, "Greys")

# Calculate distance map
distMap = PathFinding.CalculateDistanceMap(map.GetData_L2Int(), 1) # Threshold = 1 -> unknown space is not obstacle
fig, ax = plt.subplots()
Utils.PlotDistMap(map, distMap, ax)

# Calculate exploration map and exploration cells
distMapThreshold_w = 0.2
(exploreCells, explorationMap) = Exploration.CalculateExplorationMap(map, distMap, START_POS_W, distMapThreshold_w)

# Find exploration cell components
exploreCellComponents = Exploration.FindCellGraphComponents(exploreCells)

# Evaluate components
exploreCellComponentsEval = []
for i in range(len(exploreCellComponents)):
    evaluation = Exploration.EvaluateCellComponent(exploreCellComponents[i], explorationMap)
    exploreCellComponentsEval.append(evaluation)

# Plot exploration map
fig, ax = plt.subplots()
Utils.PlotDistMap(map, explorationMap, ax)

# Plot all exploration cells
for cell in exploreCells:
    cellPos_w = map.PosMapToWorld(cell)
    markPosOffset_w = 0.25 * map.GetResolution()
    ax.plot(cellPos_w[0] + markPosOffset_w, cellPos_w[1] + markPosOffset_w, "k.")
    
# Plot components of exploration cells
colors = ("g", "b", "r", "c", "m", "y", "b", "w")
for compId in range(len(exploreCellComponents)):
    comp = exploreCellComponents[compId]
    compColor = colors[compId % len(colors)]
    for cell in comp:
        cellPos_w = map.PosMapToWorld(cell)
        ax.plot(cellPos_w[0], cellPos_w[1], "o", color = compColor)
        
# Find component with biggest score
maxScore = 0
maxScoreCompId = 0 # Index of component with max score
for i in range(len(exploreCellComponentsEval)):
    compEval = exploreCellComponentsEval[i]
    if compEval[0] > maxScore:
        maxScore = compEval[0]
        maxScoreCompId = i
        
# Plot scores of all exploration cell components
for i in range(len(exploreCellComponents)):
    score = exploreCellComponentsEval[i][0]
    nearestCell = exploreCellComponentsEval[i][1]
    cellComp = exploreCellComponents[i]
    cellPos_w = map.PosMapToWorld(nearestCell)
    
    bgColor = "k"
    fontSize = "medium"
    if (i == maxScoreCompId):
        bgColor = "r"
        fontSize = "large"
    
    ax.text(cellPos_w[0], cellPos_w[1], f"{score:.2f}", color = "w",
            backgroundcolor = bgColor, fontsize = fontSize)