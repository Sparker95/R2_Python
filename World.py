# -*- coding: utf-8 -*-

import numpy as np
import math

class World:
    def __init__(self):
        self.m_aRects = np.zeros((0, 4))   # Array of rectangles: xmin, xmax, ymin, ymax
        self.m_aCircles = np.zeros((0, 3)) # Array of circles:    x, y, radius
    
    def AddRect(self, xmin, ymin, xmax, ymax):
        self.m_aRects = np.vstack((self.m_aRects, [xmin, ymin, xmax, ymax]))
    
    def AddRect2(self, xcent, ycent, w, h):
        xmin = xcent - 0.5*w
        xmax = xcent + 0.5*w
        ymin = ycent - 0.5*w
        ymax = xcent + 0.5*w
        self.m_aRects = np.vstack((self.m_aRects, [xmin, ymin, xmax, ymax]))
        
    def AddCircle(self, xcent, ycent, r):
        self.m_aCircles = np.vstack((self.m_aCircles, [xcent, ycent, r]))
        
    def GetRectangles(self):
        return self.m_aRects
    
    def GetCircles(self):
        return self.m_aCircles
    
    # Returns (xmin, ymin, xmax, ymax) of world objects
    def GetBounds(self):
        
        xmin = float(999999)
        ymin = float(999999)
        xmax = float(-999999)
        ymax = float(-999999)
        
        
        nRects = self.m_aRects.shape[0]
        for i in range(nRects):
            rect = self.m_aRects[i]
            (rxmin, rymin, rxmax, rymax) = rect
            
            if (rxmin < xmin):
                xmin = rxmin
            if (rxmax > xmax):
                xmax = rxmax
            if (rymin < ymin):
                ymin = rymin
            if (rymax > ymax):
                ymax = rymax
                
        # For now ignore circles
        
        return (xmin, ymin, xmax, ymax)
        
            
    # Returns: (bool, x, y, t)
    def TraceLine(self, line):
        nRects = self.m_aRects.shape[0]
        nCircles = self.m_aCircles.shape[0]
        allIntersections = np.full(nRects+nCircles, 2, dtype=np.float64)
        
        # Create rect formed by line as diagonal, use it for cheap pre-check of intersection
        lineRect = World.DiagonalLineToRect(line)
        
        for i in range(nRects):
            rect = self.m_aRects[i]
            if (World.XRectRect(lineRect, rect)):
                (hit, t) = World.XLineRect(line, rect)
                allIntersections[i] = t
            
        for i in range(nCircles):
            circle = self.m_aCircles[i]
            (xc, yc, r) = circle
            if (World.XRectRect(lineRect, (xc-r,yc-r,xc+r,yc+r))):
                (hit, t) = World.XLineCircle(line, circle)
                allIntersections[nRects + i] = t
            
        tmin = np.min(allIntersections) # Closest intersection
        if (tmin > 1):
            return (False, 0, 0, 2)
        
        (xhit, yhit) = World.ParametricLinePos(line, tmin)
        return (True, xhit, yhit, tmin)
        
    
    ##########################################################################
    # Helper functions for intersections
    
    # Returns coordinate of line based on parameter t
    def ParametricLinePos(line, t):
        (x1,y1, x2,y2) = line
        dx = x2-x1
        dy = y2-y1
        return (x1 + t*dx, y1 + t*dy)
    
    # Converts diagonal line into a valid rectangle
    def DiagonalLineToRect(line):
        (x0, y0, x1, y1) = line
        if (x0 > x1):
            (x0, x1) = (x1, x0)
        if (y0 > y1):
            (y0, y1) = (y1, y0)
        return (x0, y0, x1, y1)
        
    # Line to line
    # Returns: (bool, t),
    # t for line0 within: 0..1 if collided, 2 if not collided
    def XLineLine(line0, line1):
        (x1,y1, x2,y2) = line0
        (x3,y3, x4,y4) = line1
        # calculate the direction of the lines
        uA_div = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        uB_div = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        uA = 2
        if (uA_div != 0):
            uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / uA_div;
        uB = 2
        if (uB_div != 0):
            uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / uB_div;

        # if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
            #intersectionX = x1 + (uA * (x2-x1));
            #intersectionY = y1 + (uA * (y2-y1));
            return (True, uA)
        
        return (False, 2)
    
    # Line to rectangle
    # Returns: (bool, t) - lowest t
    def XLineRect(line, rect):
        (rxmin, rymin, rxmax, rymax) = rect
        
        (hitleft, tleft) = World.XLineLine(line, (rxmin, rymin, rxmin, rymax))
        (hitright, tright) = World.XLineLine(line, (rxmax, rymin, rxmax, rymax))
        (hittop, ttop) = World.XLineLine(line, (rxmin, rymax, rxmax, rymax))
        (hitbot, tbot) = World.XLineLine(line, (rxmin, rymin, rxmax, rymin))
        
        if (hitleft or hitright or hittop or hitbot):
            tmin = min(tleft, tright, ttop, tbot)
            return (True, tmin)
        
        return (False, 2)
    
    # Line to circle
    # Returns: (bool, t)
    def XLineCircle(line, circle):
        (x0, y0, x1, y1) = line
        (xc, yc, r) = circle
        
        # Shift everything so circle is at 0,0
        x0 = x0 - xc
        y0 = y0 - yc
        x1 = x1 - xc
        y1 = y1 - yc
        
        dx = x1-x0
        dy = y1-y0
        
        # Quadratic equation
        a = dx*dx + dy*dy
        b = 2*x0*dx + 2*y0*dy
        c = x0*x0 + y0*y0 - r*r
        D = b*b - 4*a*c
        
        if (D < 0):
            return (False, 2)
        
        sqrt_D = math.sqrt(D)
        
        t0 = (-b + sqrt_D)/(2*a)
        t1 = (-b - sqrt_D)/(2*a)
        
        # Remove t0, t1 outside 0..1 range
        if (t0 < 0 or t0 > 1):
            t0 = 2
        if (t1 < 0 or t1 > 1):
            t1 = 2
            
        tmin = min(t0, t1)
        if (t0 == 2 and t1 == 2): # Both are outside line segment
            return (False, 2)
        
        return (True, tmin)
    
    # Two 1D ranges
    # Returns bool
    def XRange(min0, max0, min1, max1):
        if ((min0 < max1) and (max0 > min1)):
            return True
        return False
    
    # Rectangle to rectangle
    # Returns bool
    def XRectRect(rect0, rect1):
        (r0xmin, r0ymin, r0xmax, r0ymax) = rect0
        (r1xmin, r1ymin, r1xmax, r1ymax) = rect1
        
        # Rects intersect if their limits intersect on both axes
        if (not World.XRange(r0xmin, r0xmax, r1xmin, r1xmax)):
            return False
        
        if (not World.XRange(r0ymin, r0ymax, r1ymin, r1ymax)):
            return False
        
        return True
    
# END CLASS
    
    
    
    
    
    