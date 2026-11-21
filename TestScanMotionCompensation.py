# -*- coding: utf-8 -*-
"""
Created on Thu Jan  1 15:42:32 2026

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt

import Chassis
import Utils
import SensorStreamFile
import Scan

# Read blocks from file
streamBlocks = SensorStreamFile.ReadSensorStream(r"D:\Projects\Robot2\Records\Test_Cruise_Circle.txt")

block0 = streamBlocks[40]
block1 = streamBlocks[41]

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Plot original scan
Utils.PlotScan(block1.m_ScanPolar, ax, ".b")

# Compensate scan motion
chassisConfig = Chassis.ChassisConfig()
chassisStatsConfig = Chassis.ChassisStatsModelConfig()
chassis = Chassis.ChassisModel(chassisConfig, chassisStatsConfig)

chassisInputs = Chassis.ChassisInputs()
chassisInputs.m_moveState = block1.m_MoveState
chassisInputs.m_odoDiff = Chassis.CalculateOdometerDifference(block1.m_Odometer, block0.m_Odometer)
chassisInputs.m_steer = block1.m_Steer
scan1Compensated = Scan.CompensateScanMotion(block1.m_ScanPolar, chassis, chassisInputs)

# Plot compensated scan
Utils.PlotScan(scan1Compensated, ax, ".r")