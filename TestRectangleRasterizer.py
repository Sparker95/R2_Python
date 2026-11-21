# -*- coding: utf-8 -*-
"""
Created on Sun Mar  1 21:16:31 2026

@author: spark
"""

import LineRasterizer

import matplotlib.pyplot as plt
import math
import numpy as np

dirAngle = 13 * math.pi / 180
diagCorners = np.zeros((2, 5))
horizSpans = LineRasterizer.RasterizeRect(0, 0, 4, 12, math.cos(dirAngle), math.sin(dirAngle), diagCorners)

fig, ax = plt.subplots()
ax.grid(True)

for horizSpan in horizSpans:
    (cy, minx, maxx) = horizSpan
    
    for cx in range(minx, maxx+1):
        ax.fill([cx, cx+1, cx+1, cx, cx],
                [cy, cy, cy+1, cy+1, cy],
                "b")
        
ax.plot(diagCorners[0,:], diagCorners[1,:], "-r")