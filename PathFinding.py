# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 22:24:08 2026

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt

import Map
import LineRasterizer
import Utils

# Indices of node transitions
TRANS_NONE  = 0 # Only for start node
TRANS_FWD   = 1
TRANS_FWD_L = 2
TRANS_BWD_L = 3
TRANS_BWD   = 4
TRANS_BWD_R = 5
TRANS_FWD_R = 6

# Directions of node transitions (Fwd or Bwd)
#             0  1  2   3   4   5  6
TRANS_DIR_FWD_BWD = (0, 1, 1, -1, -1, -1, 1)

class Config:
    def __init__(self):
        self.m_TurnRadius = 0.6
        self.m_NumDirs = int(16) # Number of directions. Direction index is 0..NumDirs-1.
        self.m_StepDist = 2*np.pi/self.m_NumDirs * self.m_TurnRadius # Distance of one step, always same, straight or in turn
        self.m_StepDir = 2*np.pi/self.m_NumDirs
        
        # Car dimensions (WIP later move it to ChassisConfig too)
        self.m_CarWidth = 0.22
        self.m_CarLengthFwd = 0.33
        self.m_CarLengthBwd = 0.04
        
        # Multipliers of travel cost for each transition ID
        # multFwd      = float(1.0)
        # multTurnFwd  = float(1.1)
        # multBwd      = float(4.0)
        # multTurnBwd  = float(4.2)
        multFwd      = float(1.0)
        multTurnFwd  = float(1.2)
        multBwd      = float(10.0)
        multTurnBwd  = float(12.0)
        self.m_TransIdMult = (float(1), multFwd, multTurnFwd, multTurnBwd, multBwd, multTurnBwd, multTurnFwd)
        
        # Multiplier of change of direction of travel: Fwd to Bwd or otherwise
        self.m_FwdBwdChangeMult = float(2.0)
        
        # Direction vectors of each direction index
        self.m_DirVectors = []
        for i in range(self.m_NumDirs):
            dir_rad = (2*np.pi/self.m_NumDirs)*i
            self.m_DirVectors.append((np.cos(dir_rad), np.sin(dir_rad)))

        self.m_HeuristicMult = float(5)
        self.m_OffStructureMult = 7.0 # Penalty of moving off structure map
        self.m_MaxNodes = 50000
        
        # Width of skeleton
        self.m_StructureMapWidth = 0.2

class Node:
    def __init__(self, nodeFrom = None, pose = (float(0), float(0), float(0)), transId = TRANS_NONE):
        self.m_NodeFrom = nodeFrom
        self.m_Pose = pose
        self.m_TransId = transId
        self.m_F = float(0)
        self.m_G = float(0)
        self.m_Id = int(0)

#https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
def RotatePoint(pivot, angle_rad, pt):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    
    # translate point back to origin:
    cx = pivot[0]
    cy = pivot[1]
    x = pt[0] - cx
    y = pt[1] - cy
    
    # rotate point
    xnew = x * c - y * s;
    ynew = x * s + y * c;
    
    # translate point back:
    return (xnew + cx, ynew + cy)
    

def FindNeighborNodes(config, node, outNodes):
    currentPos = (node.m_Pose[0], node.m_Pose[1])
    dir_id = node.m_Pose[2]
    vDir = config.m_DirVectors[dir_id] # Vector of current direction
    transId = node.m_TransId
    
    # First trivial cases - move straight fwd or backward
    stepDist = config.m_StepDist
    if (transId != TRANS_BWD):
        newNode = Node(node, (currentPos[0] + stepDist*vDir[0], currentPos[1] + stepDist*vDir[1], int(dir_id)), TRANS_FWD)
        outNodes.append(newNode)
    if(transId != TRANS_FWD):
        newNode = Node(node, (currentPos[0] - stepDist*vDir[0], currentPos[1] - stepDist*vDir[1], int(dir_id)), TRANS_BWD)
        outNodes.append(newNode)
    
    vSide = (vDir[1], -vDir[0])
    turnRadius = config.m_TurnRadius
    
    # Rotate around right point
    rightTurnPt = (currentPos[0] + turnRadius*vSide[0], currentPos[1] + turnRadius*vSide[1])
    if (transId != TRANS_BWD_R):
        ptRtFwd = RotatePoint(rightTurnPt, -config.m_StepDir, currentPos)
        newNode = Node(node,(ptRtFwd[0], ptRtFwd[1], (dir_id-1) % config.m_NumDirs), TRANS_FWD_R)
        outNodes.append(newNode)
    if (transId != TRANS_FWD_R):
        ptRtBwd = RotatePoint(rightTurnPt, config.m_StepDir, currentPos)
        newNode = Node(node, (ptRtBwd[0], ptRtBwd[1], (dir_id+1) % config.m_NumDirs), TRANS_BWD_R)
        outNodes.append(newNode)
    
    # Rotate around left point
    leftTurnPt = (currentPos[0] - turnRadius*vSide[0], currentPos[1] - turnRadius*vSide[1])
    if (transId != TRANS_BWD_L):
        ptLtFwd = RotatePoint(leftTurnPt, config.m_StepDir, currentPos)
        newNode = Node(node, (ptLtFwd[0], ptLtFwd[1], (dir_id+1) % config.m_NumDirs), TRANS_FWD_L)
        outNodes.append(newNode)
    if (transId != TRANS_FWD_L):
        ptLtBwd = RotatePoint(leftTurnPt, -config.m_StepDir, currentPos)
        newNode = Node(node, (ptLtBwd[0], ptLtBwd[1], (dir_id-1) % config.m_NumDirs), TRANS_BWD_L)
        outNodes.append(newNode)
    
# Return np array of positions. row 0 - x, row 1 - y
def CalculateSmoothTransition(config, nodeFrom, nodeTo):
    
    transId = nodeTo.m_TransId
    
    # Trivial case, fwd or bwd, return just pair of points without smoothing
    if transId == TRANS_FWD or transId == TRANS_BWD:
        return np.array(((nodeFrom.m_Pose[0], nodeTo.m_Pose[0]),
                        (nodeFrom.m_Pose[1], nodeTo.m_Pose[1])))
    
    # Non trivial cases    
    vDir = config.m_DirVectors[nodeFrom.m_Pose[2]]
    vSide = (vDir[1], -vDir[0])
    turnRadius = config.m_TurnRadius
    turnPointSign = 0
    if (transId == TRANS_FWD_R or transId == TRANS_BWD_R):
        turnPointSign = 1
    else:
        turnPointSign = -1
        
    turnPoint = (nodeFrom.m_Pose[0] + turnPointSign*turnRadius*vSide[0], nodeFrom.m_Pose[1] + turnPointSign*turnRadius*vSide[1]) 
    turnFromPos = (nodeFrom.m_Pose[0], nodeFrom.m_Pose[1])

    turnAngleSign = 0
    if (transId == TRANS_FWD_L or transId == TRANS_BWD_R):
        turnAngleSign = 1
    else:
        turnAngleSign = -1

    nSmoothIntervals = 5
    outPositions = np.zeros((2, nSmoothIntervals+1))
    for i in range(nSmoothIntervals+1):
        turnAngle_rad = i*turnAngleSign*config.m_StepDir/nSmoothIntervals
        smoothPos = RotatePoint(turnPoint, turnAngle_rad, turnFromPos)
        outPositions[0,i] = smoothPos[0]
        outPositions[1,i] = smoothPos[1]
        
    return outPositions
    
def Heuristic(config, thisPose, otherPose):
    dx = otherPose[0] - thisPose[0]
    dy = otherPose[1] - thisPose[1]
    return config.m_HeuristicMult*np.sqrt(dx*dx + dy*dy)

def PopMinFScoreNode(nodes):
    bestNode = nodes[0]
    bestNodeId = 0
    bestScore = bestNode.m_F
    for i in range(len(nodes)):
        node = nodes[i]
        if (node.m_F < bestScore):
            bestNode = node
            bestNodeId = i
            bestScore = node.m_F
    return nodes.pop(bestNodeId)

# Map data is in same format as in actual map
def CalculateDistanceMap(mapData, treshold = 0):
    d = mapData.copy()
    for y in range(d.shape[1]):
        for x in range(d.shape[0]):
            if d[x,y] >= treshold:
                d[x,y] = 0    # Obstacle
            else:
                d[x,y] = 9999 # Free space
    
    # Note that we ignore border row and column completely!
    for j in range(1, d.shape[1]-1, 1):
        for i in range(1, d.shape[0]-1, 1):
            e = (d[i,j-1] + 3, d[i-1,j] + 3, d[i-1,j-1] + 4, d[i-1,j+1] + 4)
            for f in e:
                if d[i,j] > f:
                    d[i,j] = f
    
    for j in range(d.shape[1]-2, 0, -1):
        for i in range(d.shape[0]-2, 0, -1):
            e = (d[i,j+1]+3, d[i+1,j]+3, d[i+1,j-1]+4, d[i+1,j+1]+4)
            for f in e:
                if d[i,j] > f:
                    d[i,j] = f
                    
    return d


def DistanceMapValueToWorld(distMapValue, mapRes):
    return (distMapValue / 3.0) * mapRes

def WorldToDistanceMapValue(worldValue, mapRes):
    return int( (worldValue / mapRes) * 3.0 )

# Map which has structure (skeleton), based on distance map
def CalculateStructureMap(config, map, distMap):
    structureMap = np.zeros(distMap.shape, dtype=int)
    
    offsetPairs = (((-1,0), (1,0)),
                   ((0,-1), (0,1)),
                   ((-1,-1), (1,1)),
                   ((-1,1), (1,-1)))
    
    for y in range(1, distMap.shape[1]-1):
        for x in range(1, distMap.shape[0]-1):
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
                
    # Make structure wider
    structureDistMap = CalculateDistanceMap(structureMap, treshold = 1)
    
    structureWidth_w = config.m_StructureMapWidth
    minDistToObstacle_w = 0.5 * config.m_CarWidth
    
    structureDistMapTreshold = 0.5 * WorldToDistanceMapValue(structureWidth_w, map.GetResolution())
    distToObstacleThreshold = WorldToDistanceMapValue(minDistToObstacle_w, map.GetResolution())
    
    for x in range(0, structureDistMap.shape[0]):
        for y in range(0, structureDistMap.shape[1]):
            if (structureDistMap[x,y] < structureDistMapTreshold and # Close to structure
                distMap[x,y] > distToObstacleThreshold): # Not too close to obstacle - physically never can be there
                structureDistMap[x,y] = 1
            else:
                structureDistMap[x,y] = 0
                
    return structureDistMap


# Returns true if car won't collise with obstacles at given pose
def CheckCollision(config, pose, map, debugPlot = False):
    
    # Debug plot
    fig = None
    ax = None
    if (debugPlot):
        fig, ax = plt.subplots()
        Utils.PlotMap(map, ax, "Greys")
        ax.plot((pose[0]), (pose[1]), "*r")
    #############
    
    dir_id = pose[2]
    pose_u = (pose[0] / map.m_res_w, pose[1] / map.m_res_w, pose[2]) # In unscaled coordinates (not world coordinates)
    
    mapPos = map.PosWorldUnscaledToMap((pose_u[0], pose_u[1]))
    
    if not map.IsMapPosWithinBounds(mapPos):
        return False
    
    # Check if this position is occupied
    if (map.m_data[mapPos[0], mapPos[1]] >= 0):
        return False
    
    # Car width fits, now check if given car orientation fits
    offsetFwd_u = 0.5*(config.m_CarLengthFwd-config.m_CarLengthBwd) / map.m_res_w
    rectCenterPos_u = (pose_u[0] + offsetFwd_u*config.m_DirVectors[dir_id][0], pose_u[1] + offsetFwd_u*config.m_DirVectors[dir_id][1])
    
    rectSpans = LineRasterizer.RasterizeRect(rectCenterPos_u[0], rectCenterPos_u[1],
                                             config.m_CarWidth/map.m_res_w, (config.m_CarLengthFwd + config.m_CarLengthBwd)/map.m_res_w,
                                             config.m_DirVectors[dir_id][0], config.m_DirVectors[dir_id][1])
    
    # Debug plot
    if (debugPlot):
        for span in rectSpans:
            (y, xStart, xEnd) = span
            y_map = y + map.m_yCenter
            xStart_map = xStart + map.m_xCenter
            xEnd_map = xEnd + map.m_xCenter
            for x_map in range(xStart_map, xEnd_map+1):
                cellPos_w = map.PosMapToWorld((x_map, y_map))
                ax.plot((cellPos_w[0]), (cellPos_w[1]), "*b")
    ##############
    
    # Iterate spans of rectangle
    for span in rectSpans:
        y_map = span[0] + map.m_yCenter      # Indices are in map's native form
        xStart_map = span[1] + map.m_xCenter
        xEnd_map = span[2] + map.m_xCenter
        
        # This is a single line (row) with start and end x coordinate, and fixed y coordinate
        # If either end doesn't fit, we're out of map
        
        if not map.IsMapPosWithinBounds((xStart_map, y_map)):
            return False
        
        if not map.IsMapPosWithinBounds((xEnd_map, y_map)):
            return False
        
        # This line span is inside map, check individual cells
        for x_map in range(xStart_map, xEnd_map + 1):
            if (map.m_data[x_map, y_map] >= 0):
                return False
        
    return True

# The main path finding function
def FindPath(config, map, startPose, destPos, destRadius):
    # Build distance map
    distMap = CalculateDistanceMap(map.m_data)
    
    # Build structure map
    structureMap = CalculateStructureMap(config, map, distMap)
    
    # Debug plot
    if (True):
        fig, ax = plt.subplots()
        Utils.PlotDistMap(map, structureMap, ax)
        fig.suptitle("Structure map")
    ################
    
    startNode = Node(pose= startPose)
    destPose = (destPos[0], destPos[1], 0)
    startNode.m_F = Heuristic(config, startPose, destPose)
    
    allNodes = [startNode]
    openSet = [startNode]
    
    # Dictionary with nodes. Key is (cellx, celly, dir_id), value is node
    nodeDict = dict()
    startNodeMapPos = map.PosWorldToMap((startNode.m_Pose[0], startNode.m_Pose[1]))
    nodeDict[(startNodeMapPos[0], startNodeMapPos[1], startPose[2])] = startNode
    
    while (len(openSet) != 0 and len(allNodes) < config.m_MaxNodes):
        currentNode = PopMinFScoreNode(openSet)
        
        if (False):
            print(f"Current node: {currentNode.m_Id} {currentNode.m_Pose[0]:.4f} {currentNode.m_Pose[1]:.4f} {currentNode.m_Pose[2]} G: {currentNode.m_G:.4f} F: {currentNode.m_F:.4f}")
        
        dx = currentNode.m_Pose[0] - destPos[0]
        dy = currentNode.m_Pose[1] - destPos[1]
        
        # We have arrived
        if (dx*dx + dy*dy < destRadius*destRadius):
            return (currentNode, allNodes)
        
        # Check collision at this node, skip it if car can't be here
        if not CheckCollision(config, currentNode.m_Pose, map):
            continue
        
        neighborNodes = []
        FindNeighborNodes(config, currentNode, neighborNodes)
        for neighborNode in neighborNodes:
            neighborNodeMapPos = map.PosWorldToMap((neighborNode.m_Pose[0], neighborNode.m_Pose[1]))
            
            # Instantly check if it's out of bounds or is instantly in obstacle
            if not map.IsMapPosWithinBounds(neighborNodeMapPos):
                continue
            if map.m_data[neighborNodeMapPos[0], neighborNodeMapPos[1]] >= 0:
                continue
            
            # However don't do full collision test yet
            
            #---------------------------------------------
            # Apply weight multipliers
            edgeWeight = 1
            edgeWeight *= config.m_TransIdMult[neighborNode.m_TransId]
            edgeWeight *= config.m_StepDist
            
            # If we step out of structure map
            if (structureMap[neighborNodeMapPos] != 1):
                edgeWeight *= config.m_OffStructureMult
                
            # If we change direction of travel: fwd to bwd or otherwise
            if (TRANS_DIR_FWD_BWD[neighborNode.m_TransId] != TRANS_DIR_FWD_BWD[currentNode.m_TransId]):
                edgeWeight *= config.m_FwdBwdChangeMult
            #---------------------------------------------
            
            neighborNodeG = currentNode.m_G + edgeWeight
            
            neighborNodeDictKey = (neighborNodeMapPos[0], neighborNodeMapPos[1], neighborNode.m_Pose[2])
            
            if neighborNodeDictKey in nodeDict:
                # We have already been there
                nodeFromDict = nodeDict[neighborNodeDictKey]
                if (nodeFromDict.m_Pose[2] != neighborNode.m_Pose[2]):
                    print("Error!")
                if (nodeFromDict.m_G > neighborNodeG):
                    nodeFromDict.m_G = neighborNodeG
                    nodeFromDict.m_F = neighborNodeG + Heuristic(config, neighborNode.m_Pose, destPose)
                    nodeFromDict.m_NodeFrom = currentNode
                    nodeFromDict.m_TransId = neighborNode.m_TransId
                # Do NOT add to allNode and openSet
            else:
                # We have never been there
                neighborNode.m_G = neighborNodeG
                neighborNode.m_F = neighborNodeG + Heuristic(config, neighborNode.m_Pose, destPose)
                neighborNode.m_Id = len(allNodes)
                # Add to allNodes and openSet and nodeDict
                allNodes.append(neighborNode)
                openSet.append(neighborNode)
                nodeDict[neighborNodeDictKey] = neighborNode
        
    # Failed to find path
    return (None, allNodes)