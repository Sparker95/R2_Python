# -*- coding: utf-8 -*-
"""
Created on Sat Apr 19 00:17:54 2025

@author: spark
"""

import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

#---------------------------------------------------------------
# Angles

# Returns angle within 0..2*pi
def WrapAngleFull(angle):
    while (angle < 0):
        angle = angle + 2*math.pi
    while (angle > 2*math.pi):
        angle = angle - 2*math.pi
    return angle

# Returns angle within -pi..pi
def WrapAngleDiff(angle):
    while (angle < -math.pi):
        angle = angle + 2*math.pi
    while (angle > math.pi):
        angle = angle - 2*math.pi
    return angle

#---------------------------------------------------------------
# Scan processing

# For full polar scan: row 0 is angles, row 1 is distances
def ScanPolarToCart(scanPolar, dirOffset = 0):
    nPoints = scanPolar.shape[1]
        
    scanOutx = []
    scanOuty = []
    
    for i in range(nPoints):
        angle = scanPolar[0, i] + dirOffset
        dist = scanPolar[1, i]
        
        if (dist <= 0):
            continue
            
        _cosAngle = np.cos(angle)
        _sinAngle = np.sin(angle)
        x = dist * _cosAngle
        y = dist * _sinAngle
        scanOutx.append(x)
        scanOuty.append(y)
    
    return np.array((scanOutx, scanOuty), dtype=float)

#---------------------------------------------------------------
# Math, matrices

# Rotation matrix
def MRot(angle):
    _sin = np.sin(angle)
    _cos = np.cos(angle)
    R = np.array([[_cos, -_sin],
                 [_sin, _cos]])
    return R

# Adds a LS pose diff to WS pose
def AddPoseDiff(pose_WS, poseDiff_LS):
    mRot = MRot(pose_WS[2])
    poseDiffXY_WS = mRot@np.array((poseDiff_LS[0], poseDiff_LS[1]))
    
    newPoseDir = WrapAngleFull(pose_WS[2] + poseDiff_LS[2])
    newPose_WS = (pose_WS[0] + poseDiffXY_WS[0], pose_WS[1] + poseDiffXY_WS[1], newPoseDir)
    return newPose_WS

# Converts log2 likelihood to float probability 0..1 
def ProbToL2Float(prob):
    return math.log2(prob / (1 - prob))

def L2FloatToProb(l2_float):
    pow2 = math.pow(2, l2_float)
    return pow2 / (1 + pow2)

def L2IntToL2Float(l2_int):
    return l2_int / float(8)

def L2FloatToL2Int(l2_float):
    return int(8 * l2_float)

#---------------------------------------------------------------
# Plotting

# Plots scan (polar)
def PlotScan(scanPolar, ax = None, plotFormat = ".b"):
    
    sensorAx = ax
    sensorFig = None
    
    if sensorAx is None:
        sensorFig, sensorAx = plt.subplots(subplot_kw={'projection': 'polar'})
            
    sensorAx.plot(scanPolar[0][:], scanPolar[1][:], plotFormat)
    sensorAx.grid(True)
    sensorAx.set_rticks(np.arange(0,5,1))
    
    return (sensorFig, sensorAx)
    
def PlotWorld(world, ax):
    patchColor = (0,0,1,0.2)
    
    rects = world.GetRectangles()
    nRects = rects.shape[0]
    for i in range(nRects):
        (xmin, ymin, xmax, ymax) = rects[i]
        rectPatch = matplotlib.patches.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, color=patchColor)
        ax.add_patch(rectPatch)
        ax.annotate(f"{i:}", xy=(0.5*(xmin+xmax),0.5*(ymin+ymax))) # Plot index of rect
    
    circles = world.GetCircles()
    nCircles = circles.shape[0]
    for i in range(nCircles):
        (x, y, radius) = circles[i]
        circlePatch = matplotlib.patches.Circle((x,y), radius, color=patchColor)
        ax.add_patch(circlePatch)
        ax.annotate(f"{i:}", xy=(x,y)) # Plot index of circle
        
def PlotLocalizer(particles, ax):
    for i in range(len(particles)):
        p = particles[i]
        
        # Current particle pose
        ax.plot(p[0], p[1], '.g', markersize = 1.5)
            
        # History of that particle
        lineSpec = '-g'
        lineWidth = 0.4
        historyPoses = np.array(p[5])
        historyPoses = np.vstack((historyPoses, (p[0], p[1], 0)))
        ax.plot(historyPoses[:,0], historyPoses[:,1], '-g', linewidth = lineWidth)
        

# https://matplotlib.org/stable/users/explain/colors/colormaps.html
def PlotMap(m, ax, colorMap = "bwr"):
    probabilities = m.GetData_Prob()
    xmin_w, ymin_w, xmax_w, ymax_w = m.GetWorldBounds()
    imshowNorm = matplotlib.colors.NoNorm(float(0), float(1), True)
    ax.imshow(np.transpose(probabilities), interpolation='nearest', origin='lower', norm = imshowNorm, cmap = colorMap, extent = (xmin_w, xmax_w, ymin_w, ymax_w))
    ax.grid(True)
    
def PlotDistMap(m, distMap, ax, vmin = 0, vmax = 64):
    
    if ax is None:
        fig, ax = plt.subplots()
    
    xmin_w, ymin_w, xmax_w, ymax_w = m.GetWorldBounds()
    ax.imshow(np.transpose(distMap), origin='lower', extent = (xmin_w, xmax_w, ymin_w, ymax_w),
              vmin = vmin, vmax = vmax)
    ax.grid(True)
    
def PlotStructMap(m, structMap, ax,):
    
    if ax is None:
        fig, ax = plt.subplots()
    
    imshowNorm = matplotlib.colors.NoNorm(float(0), float(1), True)
    xmin_w, ymin_w, xmax_w, ymax_w = m.GetWorldBounds()
    ax.imshow(np.transpose(structMap), origin='lower', extent = (xmin_w, xmax_w, ymin_w, ymax_w),
              vmin = 0, vmax = 1)
    ax.grid(True)
    
def CalculateScanPolarAngularResolution(scanPolar):
    minDiff = float(2*math.pi)
    for i in range(scanPolar.shape[1]-1):
        diff = abs(WrapAngleDiff(scanPolar[0][i] - scanPolar[0][i+1]))
        if (diff < minDiff):
            minDiff = diff
    return minDiff