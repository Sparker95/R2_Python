# -*- coding: utf-8 -*-
"""
Created on Wed Dec 31 13:26:20 2025

@author: spark
"""

import matplotlib.pyplot as plt
import numpy as np

import Chassis
import Utils

maxSteer = 0.67
# (name, x (mm), y (mm), steer, odometer diff)
measurements = [
    ("TurnRight_0", 418, -134, maxSteer, 380),
    ("TurnRight_1", 522, -235, maxSteer, 505),
    ("TurnRight_2", 214, -26, maxSteer, 189),
    ("TurnLeft_0", 176, 20, -maxSteer, 158),
    ("TurnLeft_1", 453, 184, -maxSteer, 421),
    ("TurnLeft_2", 563, 354, -maxSteer, 595)]

fig, ax = plt.subplots()
ax.grid(True)

scale = 1 / 1000

chassisConfig = Chassis.ChassisConfig()

# Plot real and estimated measurements
for m in measurements:
    xReal = scale * m[1]
    yReal = scale * m[2]
    ax.plot(xReal, yReal, "*r")
    ax.annotate(m[0], xy=(xReal, yReal))
    
    steer = m[3]
    odoDiff = m[4]
    moveState = Chassis.MOVE_STATE_FORWARD
    #chassisConfig, odoDiff, moveState, steerInput
    (xModel, yModel, dirModel) = Chassis.CalculatePoseDiff_LS(chassisConfig, odoDiff, moveState, steer)
    #ax.plot(xModel, yModel, "*b")
    
# Plot path of fixed radius turn
# Simulate pose path
odometerStep = 10
for steer in (-maxSteer, maxSteer):
    pose = (float(0), float(0), float(0))
    for odometerTtl in range(0, 600, odometerStep):
        (xModel, yModel, dirModel) = Chassis.CalculatePoseDiff_LS(chassisConfig, odometerTtl, Chassis.MOVE_STATE_FORWARD, steer)
        #ax.plot(xModel, yModel, ".b")
        
        poseDiff_LS = Chassis.CalculatePoseDiff_LS(chassisConfig, odometerStep, Chassis.MOVE_STATE_FORWARD, steer)
        newPose = Utils.AddPoseDiff(pose, poseDiff_LS)
        pose = newPose
        
        ax.plot(pose[0], pose[1], ".g")
    

