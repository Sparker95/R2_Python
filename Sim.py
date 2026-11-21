# -*- coding: utf-8 -*-
"""
Created on Sat Apr 19 16:55:17 2025

@author: spark
"""

import matplotlib.pyplot as plt
import numpy as np
import math

import World
import CarModel
import SensorModel
import ICP
import CarImaginary
import Localizer

import WorldScene
import Utils

# Various control variables
debugPlotICP = False
debugPlotLocalizerHistory = True

# Init world
world = World.World()
WorldScene.InitScene_0(world) # Populate world with objects

# Init car model
carStartPose = (2.5, 2.5, 0)
carConfig = CarModel.CarModelConfig( wheelBase = 0.3, wheelTrack = 0.2, wheelRadius = 0.03, odoNTicks = 8)
carReal = CarModel.CarModel(carConfig)
carReal.SetPose(carStartPose)

# Init sensor
sensorScanRange = float(10)
sensorNPointsInScan = 256
sensorReal = SensorModel.SensorModel(sensorScanRange, sensorNPointsInScan)

# Init robot controller
carImgStartPose = carStartPose
#localizerWorldInterface = Localizer.LocalizerInterfaceWorldReal(world, sensorReal)
carImg = CarImaginary.CarImaginary(carImgStartPose, startOdo = (0,0),
                                   carModelConfig = carConfig,
                                   sensorScanRange = sensorScanRange)

# Simulation loop
dtReal = 0.05
timeSimEnd = 5
imgSimRatio = 10 # Imaginary sim update happens even N real ones
dtImg = dtReal*imgSimRatio

# Arrays related to simualation
timesReal = np.arange(0, timeSimEnd, dtReal)
carRealPoses = [] # x,y,dir
carRealWheelAngles = [] # (L,R)
carRealOdometers = [] # (L,R)

timesImg = []
scans = []

carImgPoses = []
carImgLocalizerPoses = []

realSimCounter = 0
imgSimNumber = 0 # How many imaginary updates happened
for t in timesReal:
    print(f"Simulation: {t} / {timeSimEnd}")    
    # Store history
    carRealPose = carReal.GetPose()
    carRealPoses.append(carReal.GetPose())
    carRealWheelAngles.append(carReal.GetWheelAngles())
    carRealOdometers.append(carReal.GetOdometers())
    
    # Update imaginary car
    if (realSimCounter % imgSimRatio == 0):
        print(f"Imaginary update {imgSimNumber}")
        timesImg.append(t)
        
        # Update sensors
        scan = sensorReal.Scan(world, carRealPose)
        scans.append(scan)
        
        # Read odometry
        odoLR = carReal.GetOdometers()
        
        # Pass scans & odometry to controller
        carImg.Update(dtImg, odoLR, scan, debugPlotICP)
        
        # Record imaginary car pose
        carImgPoses.append(carImg.GetPose())
        carImgLocalizerPoses.append(carImg.GetLocalizerPose())
        
        # Plot map
        #mapFig, mapAx = plt.subplots()
        #Utils.PlotMap(carImg.GetMap().GetData(), mapAx)
        
        imgSimNumber += 1
        
    # Update real car
    steerAngle = 0
    if (t > 1 and t < 3):
        steerAngle = -15*np.pi/180.0
    if (t > 5 and t < 6.5):
        steerAngle = -20*np.pi/180.0
    if (t > 8 and t < 9.5):
        steerAngle = -13*np.pi/180.0
    if (t > 12.4 and t < 14.3):
        steerAngle = -15*np.pi/180.0        
    
    carReal.Update(dtReal, 1.0, steerAngle)    
        
    realSimCounter += 1
    
# Convert lists to np arrays
timesImg = np.array(timesImg)
carRealPoses = np.array(carRealPoses)
carRealWheelAngles = np.array(carRealWheelAngles)
carRealOdometers = np.array(carRealOdometers)
carImgPoses = np.array(carImgPoses)
carImgLocalizerPoses = np.array(carImgLocalizerPoses)
    
# Plot
fig, ax = plt.subplots()
ax.set(xlim=(0, 8), ylim=(0, 8))
ax.grid()

Utils.PlotWorld(world, ax)

ax.plot(carRealPoses[:,0], carRealPoses[:,1], '.')

# Plot where car was when imaginary update happened
for i in range(imgSimNumber):
    pose = carRealPoses[i*imgSimRatio, :]
    ax.plot(pose[0], pose[1], '*r')
    
# Plot imaginary car poses
ax.plot(carImgPoses[:,0], carImgPoses[:,1], 'or')

# Plot imaginary car poses from localizer
ax.plot(carImgLocalizerPoses[:,0], carImgLocalizerPoses[:,1], 'og')
    
# Plot best map
mapFig, mapAx = plt.subplots()
bestParticle = carImg.m_localizer.GetMostLikelyParticle()
Utils.PlotMap(bestParticle[4].GetData(), mapAx)

# Plot localizer particles history
if (debugPlotLocalizerHistory):
    for i in range(len(carImg.m_localizer.m_particlesHistory)):
        locFig, locAx = plt.subplots()
        locFig.suptitle(f"Localizer {i}")
        Utils.PlotWorld(world, locAx)
        Utils.PlotLocalizer(carImg.m_localizer.m_particlesHistory[i], locAx)
