# -*- coding: utf-8 -*-
"""
Created on Sat Jun 20 17:35:56 2026

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt
import random


#######
def SensorRayPdf(realDist, expectedDist):
    SENSOR_STD_DEV = float(0.45)
    diffNormAbs = np.abs((realDist - expectedDist) / SENSOR_STD_DEV)
    return np.pow(2, -np.pow(diffNormAbs, 4))
    
#######

N_SAMPLES = 400000

random.seed(123)

errors = []

for i in range(N_SAMPLES):
    x0 = random.uniform(-100, 100)
    y0 = random.uniform(-100, 100)
    dist = random.uniform(1, 10)
    dir_rad = random.uniform(0, 2*np.pi)
    
    xhit = x0 + dist * np.cos(dir_rad)
    yhit = y0 + dist * np.sin(dir_rad)
    
    xcell = np.floor(xhit) + 0.5
    ycell = np.floor(yhit) + 0.5
    
    dx = xcell - x0
    dy = ycell - y0
    dist_meas = np.sqrt(dx*dx + dy*dy)
    
    meas_error = dist_meas - dist
    errors.append(meas_error)
    
# Plot histogram
fig, ax = plt.subplots()
binsValues, bins, patches = ax.hist(errors, 20, (-1, 1))
ax.grid(True)

xPdfPlot = np.arange(-1, 1, 0.01)
yPdfPlot = np.zeros(xPdfPlot.shape[0])
binMaxValue = np.max(binsValues)
for i in range(xPdfPlot.shape[0]):
    x = xPdfPlot[i]
    yPdfPlot[i] = binMaxValue * SensorRayPdf(x, 0)
    
ax.plot(xPdfPlot, yPdfPlot, ".-r")
