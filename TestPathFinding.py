# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 23:41:28 2026

@author: spark
"""

import PathFinding

import matplotlib.pyplot as plt
import matplotlib

import Map
import MapUtils
import Utils

pfConfig = PathFinding.Config()
#pfConfig.m_MaxNodes = 36

# Tests for apartment
map = MapUtils.ReadMapFromFile(r"D:\Projects\Robot2\Records\Apartment.rmap")
startPose = (float(0.539), float(2.37), int(1))
destPos = (float(-4.04), float(0.84)) # under table
# destPos = (float(0), float(0), 0) # kitchen
# #destPos = (float(-3), float(-0.2), 0) # Room center
# destRadius = 0.3

# Tests for office
#map = MapUtils.ReadMapFromFile(r"D:\Projects\Robot2\Records\Legacy Encoder\Office 26 Jan 2026\Office1.rmap")
#map.Shrink()
#startPose = (float(0), float(0), int(1)) # Center of our room, facing right
#startPose = (float(0), float(0), int(0)) # Center of our room
#startPose = (float(4.4), float(-1), int(0)) # Corridor, upper part
#destPos = (float(3.04), float(-4.32), 0) # Corridor
#destPos = (float(-2.0), float(0)) # Left corner of our room
#destPos = (float(-1), float(-4.5)) # The next room
#destPos = (float(-5.3), float(-4.62)) # Other room next to table
#destPos = (float(2.25), float(-10.25))
#destPos = (float(0.3), float(-3.7)) # Other room, corner next to corridor
#destPos = (float(-5.5), float(-13.5)) # Kitchen
#destPos = (float(-0.5), float(-15.5)) # Lobby, away from structure
#destPos = (float(-1), float(-10)) # Reception desk
#destPos = (float(0), float(0), int(1)) # Center of our room
destRadius = 0.3

fig, ax = plt.subplots()

# We will plot results here, over actual map
Utils.PlotMap(map, ax, "Greys")

# # Test many iterations
# fig, ax = plt.subplots()
# ax.grid(True)
# startNode = PathFinding.Node()
# #startNode.m_TransId = PathFinding.TRANS_BWD_L
# allNodes = [startNode] # All nodes created, for tracking memory
# front = [startNode]
# iterCounter = 0
# while iterCounter < 1:
#     newFront = []
    
#     for node in front:
#         newNodes = []
#         PathFinding.FindNeighborNodes(pfConfig, node, allNodes, newNodes)
#         newFront.extend(newNodes)
        
#         for newNode in newNodes:
#             ax.plot((node.m_Pose[0], newNode.m_Pose[0]), (node.m_Pose[1], newNode.m_Pose[1]), ".-b")
            
#     # Swap front and newFront
#     front = newFront
    
#     iterCounter += 1





(endNode, allNodes) = PathFinding.FindPath(pfConfig, map, startPose, destPos, destRadius)

# Process the results
ax.grid(True)
patches = []
patches.append(matplotlib.patches.Circle(destPos, destRadius))
patchCollection = matplotlib.collections.PatchCollection(patches, alpha = 0.5)
ax.add_collection(patchCollection)

# Recover list of nodes
pathNodesReversed = []
if not (endNode is None):
    iterNode = endNode
    while (True):
        pathNodesReversed.append(iterNode)
        fromNode = iterNode.m_NodeFrom
        if fromNode is None:
            break
        iterNode = fromNode

# List of nodes in normal order (start to end)
pathNodes = list(reversed(pathNodesReversed))

for i in range(len(pathNodes) - 1):
    iterNode = pathNodes[i]
    nextNode = pathNodes[i+1]
    # Plot small arrow pointing where node is directed
    arrowLength = 0.075
    dirVector = pfConfig.m_DirVectors[iterNode.m_Pose[2]]
    arrowEndPos = (iterNode.m_Pose[0] + arrowLength * dirVector[0],
                   iterNode.m_Pose[1] + arrowLength * dirVector[1])
    ax.plot((iterNode.m_Pose[0], arrowEndPos[0]), (iterNode.m_Pose[1], arrowEndPos[1]), "r")

    # Plot dots and nodes' end points
    ax.plot((iterNode.m_Pose[0]), (iterNode.m_Pose[1]), ".b")
    ax.plot((nextNode.m_Pose[0]), (nextNode.m_Pose[1]), ".b")
    
    # Calculate and plot smooth path between nodes
    smoothPositions = PathFinding.CalculateSmoothTransition(pfConfig, iterNode, nextNode)
    ax.plot(smoothPositions[0,:], smoothPositions[1,:], "-b")
        
# Plot all positions it explored
if (False):
    for node in allNodes:
        ax.plot((node.m_Pose[0]), (node.m_Pose[1]), ".-r")