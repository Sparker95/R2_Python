# -*- coding: utf-8 -*-
"""
Created on Sat Dec 27 12:24:18 2025

@author: spark
"""

import FileUtils
import StringWordReader

import numpy as np
import matplotlib.pyplot as plt

class LocalizerHistoryEntryParticle:
    def __init__(self):
        self.m_Pose = (float(0), float(0), float(0))
        self.m_Weight = float(0)
        self.m_UniquePoseHistory = np.zeros((3, 0))

class LocalizerHistoryEntry:
    def __init__(self):
        self.m_ID = int(0)
        self.m_AfterResampling = bool(False)
        self.m_CommonPoseHistory = np.zeros((3,0))
        self.m_Particles = []
        
    def GetMostLikelyParticle(self):
        maxWeight = self.m_Particles[0].m_Weight
        bestParticle = self.m_Particles[0]
        for p in self.m_Particles:
            if p.m_Weight > maxWeight:
                maxWeight = p.m_Weight
                bestParticle = p
        return bestParticle
                

def _ReadLocalizerHistoryEntry(reader):
    r = reader
    
    historyEntry = LocalizerHistoryEntry()
    
    r.AssertWord("HISTORY_ENTRY")
    
    r.AssertWord("ENTRY_ID")
    historyEntry.m_ID = r.ReadInt()
    
    r.AssertWord("AFTER_RESAMPLING")
    historyEntry.m_AfterResampling = r.ReadBool()
    
    r.AssertWord("COMMON_POSE_HISTORY")
    r.AssertWord("N_POSES")
    nCommonPoses = r.ReadInt()
    commonPoseHistory = np.zeros((3, nCommonPoses))
    for commonPoseId in range(nCommonPoses):
        r.AssertWord("POSE")
        commonPoseHistory[0, commonPoseId] = r.ReadFloat()
        commonPoseHistory[1, commonPoseId] = r.ReadFloat()
        commonPoseHistory[2, commonPoseId] = r.ReadFloat()
    historyEntry.m_CommonPoseHistory = commonPoseHistory
    
    r.AssertWord("PARTICLES")
    r.AssertWord("N_PARTICLES")
    nParticles = r.ReadInt()
    
    for i in range(nParticles):
        particle = LocalizerHistoryEntryParticle()
        r.AssertWord("PTCL")
        
        r.AssertWord("POSE")
        particle.m_Pose = (r.ReadFloat(), r.ReadFloat(), r.ReadFloat())
        
        r.AssertWord("WT")
        particle.m_Weight = r.ReadFloat()
        
        r.AssertWord("UNIQUE_POS_HIST")
        
        r.AssertWord("N_POSES")
        nPoses = r.ReadInt()
        poseHistory = np.zeros((3,nPoses + 2)) # !!! First and last poses are added as well, it's needed for plotting !!!
        poseHistory[:, 0] = commonPoseHistory[:, -1]
        
        for poseId in range (nPoses):
            r.AssertWord("POSE")
            poseHistory[0][poseId + 1] = r.ReadFloat() # x
            poseHistory[1][poseId + 1] = r.ReadFloat() # y
            poseHistory[2][poseId + 1] = r.ReadFloat() # dir
        poseHistory[0][-1] = particle.m_Pose[0]
        poseHistory[1][-1] = particle.m_Pose[1]
        poseHistory[2][-1] = particle.m_Pose[2]
        particle.m_UniquePoseHistory = poseHistory
        historyEntry.m_Particles.append(particle)
        
    return historyEntry

def ReadLocalizerHistoryFromFile(filePath):
    fileContent = FileUtils.ReadFileIntoString(filePath)
    
    r = StringWordReader.StringWordReader(fileContent)
    
    r.AssertWord("LOCALIZER_HISTORY")
    r.AssertWord("VERSION")
    r.AssertWord("2")
    r.AssertWord("HISTORY_ENTRIES")
    
    historyEntries = []
    while (not r.End()):
        historyEntry = _ReadLocalizerHistoryEntry(r)
        historyEntries.append(historyEntry)
        
    return historyEntries

def PlotLocalizerHistoryEntry(historyEntry, ax = None):
    
    if ax is None:
        fig, ax = plt.subplots()
        fig.suptitle("ID: {0}, After Resampling: {1}".format(historyEntry.m_ID, historyEntry.m_AfterResampling))
        ax.set_aspect("equal", adjustable="box")
    
    
    ax.grid(True)
    
    # Common history of all particles
    ax.plot(historyEntry.m_CommonPoseHistory[0,:], historyEntry.m_CommonPoseHistory[1,:], "-b", linewidth = 0.8)
    
    for i in range(len(historyEntry.m_Particles)):
        p = historyEntry.m_Particles[:][i]
        
        # Current particle pose
        ax.plot(p.m_Pose[0], p.m_Pose[1], '.g', markersize = 1.5)
        
        # Unique history of that particle
        if (p.m_UniquePoseHistory.shape[1] != 0):
            ax.plot(p.m_UniquePoseHistory[0,:], p.m_UniquePoseHistory[1,:], "-g", linewidth = 0.4)
        