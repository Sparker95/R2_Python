# -*- coding: utf-8 -*-
"""
Created on Thu Apr 24 20:41:15 2025

@author: spark
"""

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt

import Utils

import Map

m = Map.Map(8, 8, 0.1)

oldData = m.m_data
#oldData[0,0] = 0
#oldData[oldData.shape[0]-1,0] = 0
#oldData[oldData.shape[0]-1, oldData.shape[1]-1] = 127
#oldData[0, oldData.shape[1]-1] = 0
oldData[m.m_xCenter, m.m_yCenter] = -127
oldData[m.m_xCenter+2, m.m_yCenter+1] = 50


fig, ax = plt.subplots()
Utils.PlotMap(m, ax)

m.EnsureWorldBounds_Rect(0.1, -0.6, 0.6, 0.6)

newData = m.m_data

fig, ax = plt.subplots()
Utils.PlotMap(m, ax)
