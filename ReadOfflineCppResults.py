# -*- coding: utf-8 -*-
"""
Created on Sat Dec 27 12:48:06 2025

@author: spark
"""

import LocalizerUtils
import MapUtils

import matplotlib.pyplot as plt

# From r2offline_pi
#historyFilePath = r"D:\Projects\Robot2\CPP\r2offline_pi\Results\TestLocalizerHistory.rlhist"
#mapFilePath = r"D:\Projects\Robot2\CPP\r2offline_pi\Results\TestMap.rmap"

# From local r2offline_win
historyFilePath = r"D:\Projects\Robot2\CPP\r2offline_win\TestLocalizerHistory.rlhist"
mapFilePath = r"D:\Projects\Robot2\CPP\r2offline_win\TestMap.rmap"

historyEntries = LocalizerUtils.ReadLocalizerHistoryFromFile(historyFilePath)

plt.ioff()

# Plots history for each entry in new window
if (False):
    for entry in historyEntries:
        LocalizerUtils.PlotLocalizerHistoryEntry(entry)

mapFig, mapAx = MapUtils.PlotMapFromFile(mapFilePath, "Greys")

# Plot path of most likely particle
if (len(historyEntries) != 0):
    plotFormat = ".-b"
    linewidth = 0.4
    markersize = 1.5
    lastHistoryEntry = historyEntries[-1]
    mapAx.plot(lastHistoryEntry.m_CommonPoseHistory[0,:], lastHistoryEntry.m_CommonPoseHistory[1,:], plotFormat,
               linewidth = linewidth, markersize = markersize)
    bestParticle = lastHistoryEntry.GetMostLikelyParticle()
    mapAx.plot(bestParticle.m_UniquePoseHistory[0,:], bestParticle.m_UniquePoseHistory[1,:], plotFormat,
               linewidth = linewidth, markersize = markersize)

plt.show()