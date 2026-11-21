# -*- coding: utf-8 -*-
"""
Created on Mon Apr 28 23:31:25 2025

@author: spark
"""

import matplotlib.pyplot as plt

import World
import WorldScene
import Localizer
import Utils

import SensorModel

world = World.World()
WorldScene.InitScene_0(world)

sensor = SensorModel.SensorModel(float(10), 128)
worldInterface = Localizer.LocalizerInterfaceWorldReal(world, sensor)
loc = Localizer.Localizer(256, worldInterface)

loc.InitParticles()


fig, ax = plt.subplots()
ax.set(xlim=(0, 7), ylim=(0, 8))
ax.grid()

Utils.PlotWorld(world, ax)

Utils.PlotLocalizer(loc, ax)