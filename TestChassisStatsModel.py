# -*- coding: utf-8 -*-
"""
Created on Sun Dec 28 18:25:47 2025

@author: spark
"""

import matplotlib.pyplot as plt

import Chassis
import Utils

chassisConfig = Chassis.ChassisConfig()
chassisStatsConfig = Chassis.ChassisStatsModelConfig()

chassisStatsModel = Chassis.ChassisStatsModel(chassisConfig, chassisStatsConfig)

nPoses = 32
nIterations = 20

odoDiff = 42
steerInput = float(1.0)

poses = []
for i in range (nPoses):
    poses.append((float(0), float(0), float(0)))
    
fig, ax = plt.subplots()
ax.grid(True)
ax.set_aspect('equal', adjustable='box')
plt.ioff()

def PlotPoses(poses, ax):
    for pose in poses:
        ax.plot(pose[0], pose[1], '.b', markersize = 1.5)

for i in range(nIterations):
    PlotPoses(poses, ax)
    
    # Advance poses
    for poseId in range(nPoses):
        pose = poses[poseId]
        poseDiff_LS = chassisStatsModel.GetPoseDiffSample_LS(odoDiff, Chassis.MOVE_STATE_BACKWARD, steerInput)
        newPose = Utils.AddPoseDiff(pose, poseDiff_LS)
        poses[poseId] = newPose
    
plt.show()