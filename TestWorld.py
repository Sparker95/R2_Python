# -*- coding: utf-8 -*-
"""
Created on Tue Apr 15 23:45:55 2025

@author: spark
"""

import matplotlib.pyplot as plt
import numpy as np
import math

import World
import SensorModel

# Init world
w = 0.5 # Wall thickness
world = World.World()
world.AddRect(0, 0, w, 7)  # 0
world.AddRect(0, 7-w, 6, 7)
world.AddRect(6-w, 0, 6, 7)
world.AddRect(0, 0, 6, w)
world.AddRect(4.2, 1, 4.5, 1.3) # 4
world.AddRect(3, 3.3, 3.3, 4.3)
world.AddRect(0, 4.3, 4, 4.3+w)
world.AddRect(0, 4.8, 1.5, 5.3)
world.AddRect(5.25, 4.3, 6, 4.3+w) #8

world.AddCircle(1.5, 1.5, 0.5)
world.AddCircle(3, 6, 0.25)
world.AddCircle(5, 2, 0.1)
world.AddCircle(4.75, 2.5, 0.1)

# Init plot
fig, ax = plt.subplots()
ax.set(xlim=(0, 7), ylim=(0, 7))
ax.grid()

# Plot world
world.PlotObjects(ax)

# Test intersection
"""
nScans = 256
for i in range(nScans):
    angle = i/nScans*2*math.pi
    lx0 = 4.8
    lx1 = 4
    llen = 7
    line = (lx0, lx1, lx0+llen*math.cos(angle), lx1+llen*math.sin(angle))
    (hit, xhit, yhit) = world.TraceLine(line)
    ax.plot([line[0],line[2]], [line[1], line[3]], '-b', linewidth=0.2)
    if (hit):
        ax.plot(xhit, yhit, '*r')
"""

# Test sensor
sensorNPoints = 512
sensorPosx = 1
sensorPosy = 4
sensorDir = np.pi/2.0
sensor = SensorModel.Sensor(6, sensorNPoints)
scanDists = sensor.Scan(world, sensorPosx, sensorPosy, sensorDir)
scanAngles = np.zeros(sensorNPoints)
for i in range(sensorNPoints):
    scanAngles[i] = i*2.0*np.pi/sensorNPoints
sensorFig, sensorAx = plt.subplots(subplot_kw={'projection': 'polar'})
for i in range(sensorNPoints):
    if (scanDists[i] < 0):
        scanDists[i] = 0
sensorAx.plot(scanAngles, scanDists, '.')
sensorAx.grid(True)
sensorAx.set_rticks(np.arange(0,5,1))

ax.plot(sensorPosx, sensorPosy, '*r')


plt.show()