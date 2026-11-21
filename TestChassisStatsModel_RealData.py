# -*- coding: utf-8 -*-
"""
Created on Sun Dec 28 18:25:47 2025

@author: spark
"""

import matplotlib.pyplot as plt

import Chassis
import Utils
import SensorStreamFile

streamBlocks = SensorStreamFile.ReadSensorStream(r"RealData/Test_Cruise_Circle.txt")

chassisConfig = Chassis.ChassisConfig()
chassisStatsConfig = Chassis.ChassisStatsModelConfig()

chassisStatsModel = Chassis.ChassisStatsModel(chassisConfig, chassisStatsConfig)

nPoses = 100
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

# https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.plot.html
formats = [".r", ".g", ".b", ".m", ".k"]

def PlotPoses(poses, ax, fmt = ".b"):
    for pose in poses:
        ax.plot(pose[0], pose[1], fmt, markersize = 1.5)

plotId = int(0)
for blockId in range(1, len(streamBlocks), 1):
    
    if blockId % 5 == 0:
        fmtId = plotId % len(formats)
        PlotPoses(poses, ax, formats[fmtId])
        plotId = plotId + 1
    
    # Advance poses
    prevBlock = streamBlocks[blockId-1]
    block = streamBlocks[blockId]
    odoDiff = block.m_Odometer - prevBlock.m_Odometer
    steerInput = block.m_Steer
    moveState = block.m_MoveState
    for poseId in range(nPoses):
        pose = poses[poseId]
        poseDiff_LS = chassisStatsModel.GetPoseDiffSample_LS(odoDiff, moveState, steerInput)
        newPose = Utils.AddPoseDiff(pose, poseDiff_LS)
        poses[poseId] = newPose
    
plt.show()


# fig, ax = plt.subplots()
# ax.grid(True)
# ax.set_aspect("equal", adjustable="box")
# fig.suptitle("Block {0}".format(blockId))