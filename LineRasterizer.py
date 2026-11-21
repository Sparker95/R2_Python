# -*- coding: utf-8 -*-
"""
LineRasterizer like in LineRasterizer.h & .cpp
Created on Sun Mar  1 17:32:57 2026

@author: spark
"""

import math
import numpy as np

class LineRasterizer:
    def __init__(self, startx, starty, dirx, diry):
        self.m_x = float(startx)
        self.m_y = float(starty)
        self.m_dist = float(0)
        if abs(dirx) > abs(diry):
            self.m_incx = math.copysign(1, dirx)
            self.m_incy = math.copysign(diry / dirx, diry)
        else:
            self.m_incx = math.copysign(dirx / diry, dirx)
            self.m_incy = math.copysign(1, diry)
            
        self.m_incDist = math.sqrt(self.m_incx * self.m_incx + self.m_incy * self.m_incy)
        
    def Inc(self):
        self.m_x += self.m_incx
        self.m_y += self.m_incy
        self.m_dist += self.m_incDist

def RasterizeLine(startx, starty, endx, endy):
    dirx = endx - startx
    diry = endy - starty
    lineLength = math.sqrt(dirx*dirx + diry * diry)
    dirx /= lineLength
    diry /= lineLength
    r = LineRasterizer(startx, starty, dirx, diry)
    
    outLine = []
    
    while True:
        
        if r.m_dist < lineLength:    
            cellx = math.floor(r.m_x)
            celly = math.floor(r.m_y)
            outLine.append((cellx, celly))
            r.Inc()
        else:
            # Check if last pixel should be added
            cellx = math.floor(endx)
            celly = math.floor(endy)
            
            if (len(outLine) != 0):
                prevCell = outLine[-1]
                if (prevCell[0] != cellx or prevCell[1] != celly):
                    outLine.append((cellx, celly))
            else:
                outLine.append((cellx, celly))
            break
    
    return outLine

# Returns horizontal spans: list of (y, xStart, xEnd)
def RasterizeRect(centerx, centery, width, length, dirx, diry, outCorners = None):
    dl = (dirx, diry)   # Direction along length
    dw = (diry, -dirx)  # Direction along width
    
    hw = 0.5 * width  # Half width
    hl = 0.5 * length # Half height
    
    # Calculate coordinates of all corners
    cornerSigns = ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))
    corners = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
    
    for cornerId in range(5):
        cs = cornerSigns[cornerId]
        cornerx = centerx + cs[1] * hw * dw[0] + cs[0] * hl * dl[0]
        cornery = centery + cs[1] * hw * dw[1] + cs[0] * hl * dl[1]
        corners[cornerId][0] = cornerx
        corners[cornerId][1] = cornery
        
    if not outCorners is None:
       for cornerId in range(5):
           outCorners[0, cornerId] = corners[cornerId][0]
           outCorners[1, cornerId] = corners[cornerId][1]
    
    # Rasterize lines between all corners
    allBorderCells = []
    for cornerId in range(4):
        nextCornerId = cornerId + 1
        borderLineCells = RasterizeLine(corners[cornerId][0], corners[cornerId][1], corners[nextCornerId][0], corners[nextCornerId][1])
        allBorderCells.extend(borderLineCells)
        
    # Find min, max x,y of border cells
    minx = allBorderCells[0][0]
    maxx = minx
    miny = allBorderCells[0][1]
    maxy = miny
    for cell in allBorderCells:
        (x, y) = (cell[0], cell[1])
        if x < minx:
            minx = x
        if x > maxx:
            maxx = x
        if y < miny:
            miny = y
        if y > maxy:
            maxy = y
    
    # Create horizontal spans
    nHorizLines = maxy - miny + 1
    spanMin = maxx * np.ones(nHorizLines, dtype=int)
    spanMax = minx * np.ones(nHorizLines, dtype=int)
    
    for cell in allBorderCells:
        (x, y) = (cell[0], cell[1])
        vertId = y - miny
        
        if x < spanMin[vertId]:
            spanMin[vertId] = x
        if x > spanMax[vertId]:
            spanMax[vertId] = x
            
    # Return horizontal spans: list of (y, xStart, xEnd)
    outSpans = []
    for lineId in range(nHorizLines):
        outSpans.append(((lineId + miny), spanMin[lineId], spanMax[lineId]))
        
    return outSpans
