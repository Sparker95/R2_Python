# -*- coding: utf-8 -*-
"""
Created on Sun Apr 20 14:32:46 2025

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial
import Utils

# Jacobian matrix
def MJacobian(point, rotDer):
    J = np.zeros((2,3), dtype=float)
    J[0,0] = 1.0
    J[1,1] = 1.0
    J_col2 = rotDer @ point
    J[:,2] = J_col2
    return J

# Derivative of rotation matrix
def MRotDerivative(angle):
    _sin = np.sin(angle)
    _cos = np.cos(angle)
    RDer = np.array([[-_sin, -_cos],
                    [_cos, -_sin]])
    return RDer
    
# Returns array of ints. Id - index of pt in scan0. value[id] - index of pt in scan1.
def GetPointCorrespondence(maxDist0, scan0, maxDist1, scan1, scan1Tree):
    nPoints0 = scan0.shape[1]
    
    corr = np.arange(0, nPoints0, 1, dtype=int)
    
    for i in range(nPoints0):
        pt0 = scan0[:,i]
        
        # Iterate pts in scan1, find nearest
        (minDist, nearestPtId) = scan1Tree.query(pt0)
        
        threshold = 3 * np.max((maxDist0[i], maxDist1[nearestPtId]))
        if (minDist < threshold):
            corr[i] = nearestPtId
        else:
            corr[i] = -1 # Too far away
        
    return corr
                

def WeightFunc(diff, threshold):
    (dx,dy) = diff
    d = np.sqrt(dx*dx + dy*dy)
    if (d < threshold):
        return float(1.0)
    return threshold/d

def WeightFunc2(diff, threshold, distFromScanOrigin):
    (dx,dy) = diff
    d = np.sqrt(dx*dx + dy*dy)
    if (d < threshold):
        return distFromScanOrigin
    return distFromScanOrigin * threshold / d

# Returns transform T: (x, y, dir)
# WIP delete this later, old scan format is not supported
def MatchScans(scan0Polar, scan1Polar, nIterMax, initialT=(0,0,0), debugPlot=False):
    # https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
    # https://medium.com/@nathan.myron/gauss-newton-method-7dbb3d4ac32c
    # https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
    
    (scan0PolarFiltered, scan0) = Utils.ScanPolarToCart(scan0Polar)
    (scan1PolarFiltered, scan1) = Utils.ScanPolarToCart(scan1Polar)
    scan1Tree = scipy.spatial.KDTree(scan1.T)
    
    distThresholdCoeff = float(1.0)
    maxDist0 = distThresholdCoeff*np.pi/float(scan0Polar.shape[0])*scan0Polar
    maxDist1 = distThresholdCoeff*np.pi/float(scan1Polar.shape[0])*scan1Polar
    
    nPoints = scan0.shape[1]
    weights = np.zeros(nPoints, dtype=float)

    ## Plotting
    plt_nAxesHoriz, plt_fig, plt_axesGrid, plt_axes = (0,0,0,0)
    if (debugPlot):
        print("ICP Start")
        plt_nAxesHoriz = int(np.ceil(np.sqrt(nIterMax)))
        plt_fig, plt_axesGrid = plt.subplots(plt_nAxesHoriz, plt_nAxesHoriz)
        plt_axes = []
        for axesArray in plt_axesGrid :
            for a in axesArray:
                plt_axes.append(a)
    ## 

    # Current estimate 
    T = np.array(initialT, dtype=float)
    
    for iteration in range(nIterMax):
        mRot = Utils.MRot(T[2])
        scan0Transformed = mRot@scan0                        # Rotate
        scan0Transformed[0,:] = scan0Transformed[0,:] + T[0] # Translate
        scan0Transformed[1,:] = scan0Transformed[1,:] + T[1]
        
        corresp = GetPointCorrespondence(scan0PolarFiltered, scan0Transformed,
                                         scan1PolarFiltered, scan1,
                                         scan1Tree)
        
        # Calculate errors
        scanErrors = np.zeros((2, nPoints)) # Matches scan0 array
        for i in range(nPoints):
            if(corresp[i] == -1): # It means outlier doesn't contribute to JxError
                continue
            
            e = scan0Transformed[:, i] - scan1[:, corresp[i]]
            
            # Correct error by weight
            weightThreshold = np.max((maxDist0[i], maxDist1[corresp[i]]))
            distToPt = np.max((scan0Polar[i], scan1Polar[corresp[i]]))
            weight = WeightFunc2(e, weightThreshold, distToPt)
            weights[i] = weight
            
            scanErrors[:, i] =  weight * e
            
        
        # Calculate Jacobian, Hessian
        JxError = np.zeros(3, dtype=float)
        H = np.zeros((3,3), dtype=float)
        
        mRotDer = MRotDerivative(T[2])
        for i in range(nPoints):
            Ji = MJacobian(scan0Transformed[:, i], mRotDer)
            JxError = JxError + (Ji.T @ scanErrors[:,i]) # Error already multiplied by weight
            H = H + weights[i] * (Ji.T @ Ji)
        
        deltaT = np.linalg.solve(H, -JxError)
        T = T + deltaT
        
        scanError = (np.sum(scanErrors[0,:]),
                   np.sum(scanErrors[1,:]))
        
        ## Plotting
        if (debugPlot):
            print(f"Iteration: {iteration}, error: {scanError}, T: {T}")
            plt_ax = plt_axes[iteration]
            plt_ax.plot(scan1[0,:],scan1[1,:], '.r', markersize = 2)
            for i in range(nPoints):
                fmt = ".b"
                mrksize = 2
                if (corresp[i] == -1):
                    fmt = "vb" # Outlier
                    mrksize = 3
                plt_ax.plot(scan0Transformed[0,i], scan0Transformed[1,i], fmt, markersize = mrksize)
                
                # Plot weight
                weight = weights[i]
                plt_ax.annotate(f"{weight:.3f}", xy=(scan0Transformed[0,i], scan0Transformed[1,i]))
            
            # Plot point correspondance
            for i in range(nPoints):
                if (corresp[i] == -1):
                    continue
                
                pt0 = scan0Transformed[:, i]
                pt1 = scan1[:, corresp[i]]
                plt_ax.plot( (pt0[0], pt1[0]), (pt0[1], pt1[1]), '-b', linewidth = 0.5 )
            
            plt_ax.set_title(f"Iter: {iteration}")
        ##
        
    return T

# Returns transform T: (x, y, dir)
def MatchFullPolarScans(scan0Polar, scan1Polar, nIterMax, initialT=(0,0,0), debugPlot=False):
    # https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb
    # https://medium.com/@nathan.myron/gauss-newton-method-7dbb3d4ac32c
    # https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
    
    scan0 = Utils.ScanPolarToCart(scan0Polar)
    scan1 = Utils.ScanPolarToCart(scan1Polar)
    scan1Tree = scipy.spatial.KDTree(scan1.T)
    
    distThresholdCoeff = float(1.0)
    angularRes0 = Utils.CalculateScanPolarAngularResolution(scan0Polar)
    angularRes1 = Utils.CalculateScanPolarAngularResolution(scan1Polar)
    maxDist0 = distThresholdCoeff * angularRes0 * scan0Polar[1, :]
    maxDist1 = distThresholdCoeff * angularRes1 * scan1Polar[1, :]
    
    nPoints = scan0.shape[1]
    weights = np.zeros(nPoints, dtype=float)

    ## Plotting
    plt_nAxesHoriz, plt_fig, plt_axesGrid, plt_axes = (0,0,0,0)
    if (debugPlot):
        print("ICP Start")
        plt_nAxesHoriz = int(np.ceil(np.sqrt(nIterMax)))
        plt_fig, plt_axesGrid = plt.subplots(plt_nAxesHoriz, plt_nAxesHoriz)
        plt_axes = []
        for axesArray in plt_axesGrid :
            for a in axesArray:
                plt_axes.append(a)
    ## 

    # Current estimate 
    T = np.array(initialT, dtype=float)
    
    for iteration in range(nIterMax):
        mRot = Utils.MRot(T[2])
        scan0Transformed = mRot@scan0                        # Rotate
        scan0Transformed[0,:] = scan0Transformed[0,:] + T[0] # Translate
        scan0Transformed[1,:] = scan0Transformed[1,:] + T[1]
        
        corresp = GetPointCorrespondence(maxDist0, scan0Transformed,
                                         maxDist1, scan1,
                                         scan1Tree)
        
        # Calculate errors
        scanErrors = np.zeros((2, nPoints)) # Matches scan0 array
        for i in range(nPoints):
            if(corresp[i] == -1): # It means outlier doesn't contribute to JxError
                continue
            
            e = scan0Transformed[:, i] - scan1[:, corresp[i]]
            
            # Correct error by weight
            weightThreshold = np.max((maxDist0[i], maxDist1[corresp[i]]))
            distToPt = np.max((scan0Polar[1, i], scan1Polar[1, corresp[i]]))
            weight = WeightFunc2(e, weightThreshold, distToPt)
            weights[i] = weight
            
            scanErrors[:, i] =  weight * e
            
        
        # Calculate Jacobian, Hessian
        JxError = np.zeros(3, dtype=float)
        H = np.zeros((3,3), dtype=float)
        
        mRotDer = MRotDerivative(T[2])
        for i in range(nPoints):
            Ji = MJacobian(scan0Transformed[:, i], mRotDer)
            JxError = JxError + (Ji.T @ scanErrors[:,i]) # Error already multiplied by weight
            H = H + weights[i] * (Ji.T @ Ji)
        
        deltaT = np.linalg.solve(H, -JxError)
        T = T + deltaT
        
        scanError = (np.sum(scanErrors[0,:]),
                   np.sum(scanErrors[1,:]))
        
        ## Plotting
        if (debugPlot):
            print(f"Iteration: {iteration}, error: {scanError}, T: {T}")
            plt_ax = plt_axes[iteration]
            plt_ax.plot(scan1[0,:],scan1[1,:], '.r', markersize = 2)
            for i in range(nPoints):
                fmt = ".b"
                mrksize = 2
                if (corresp[i] == -1):
                    fmt = "vb" # Outlier
                    mrksize = 3
                plt_ax.plot(scan0Transformed[0,i], scan0Transformed[1,i], fmt, markersize = mrksize)
                
                # Plot weight
                plotWeight = False
                if (plotWeight):
                    weight = weights[i]
                    plt_ax.annotate(f"{weight:.3f}", xy=(scan0Transformed[0,i], scan0Transformed[1,i]))
            
            # Plot point correspondance
            for i in range(nPoints):
                if (corresp[i] == -1):
                    continue
                
                pt0 = scan0Transformed[:, i]
                pt1 = scan1[:, corresp[i]]
                plt_ax.plot( (pt0[0], pt1[0]), (pt0[1], pt1[1]), '-b', linewidth = 0.5 )
            
            # Plot maxDist
            plotMaxDist = True
            if (plotMaxDist):
                # scan 0
                for i in range(scan0.shape[1]):
                    circle = plt.Circle(scan0Transformed[:,i], maxDist0[i], color='b', fill=False)
                    plt_ax.add_patch(circle)
                
                # scan 1
                for i in range(scan1.shape[1]):
                    circle = plt.Circle(scan1[:,i], maxDist1[i], color='r', fill=False)
                    plt_ax.add_patch(circle)
                
            
            plt_ax.set_title(f"Iter: {iteration}")
        ##
        
    return T

