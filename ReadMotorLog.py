# -*- coding: utf-8 -*-
"""
Created on Sat Feb 21 14:15:13 2026

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt

filePath = "MotorLog.txt"

f = open(filePath, "rt")

logData = []

while (True):
    l = f.readline(1024)
    if (len(l) == 0):
        break;
    
    if (len(l) < 10): # Ignore "ACK" line
        continue;
        
    # LOG 200.0 186.0 0.140 0.982 0.203 FWD
    tokens = l.split(" ")
    
    tgtFreq = float(tokens[1])
    freq = float(tokens[2])
    error = float(tokens[3])
    errorIntegral = float(tokens[4])
    output = float(tokens[5])
    
    logData.append([tgtFreq, freq, error, errorIntegral, output])
    
logDataSize = len(logData)

time = np.zeros(logDataSize)
tgtFreq = np.zeros(logDataSize)
freq = np.zeros(logDataSize)
error = np.zeros(logDataSize)
errorIntegral = np.zeros(logDataSize)
output = np.zeros(logDataSize)

for i in range(logDataSize):
    d = logData[i]
    tgtFreq[i] = d[0]
    freq[i] = d[1]
    error[i] = d[2]
    errorIntegral[i] = d[3]
    output[i] = d[4]
    time[i] = i / 50
    
# Plot data

fig, ax = plt.subplots(2,2)

ax[0,0].plot(time, tgtFreq, ".-r")
ax[0,0].plot(time, freq, ".-b")
ax[0,0].grid(True)
ax[0,0].set_title("Target Frequency (red), Frequency (Blue)")

ax[1,0].plot(time, error, '.-b')
ax[1,0].grid(True)
ax[1,0].set_title("Error (Scaled)")

ax[0,1].plot(time, errorIntegral, '.-b')
ax[0,1].grid(True)
ax[0,1].set_title("Error Integral")

ax[1,1].plot(time, output, ".-b")
ax[1,1].grid(True)
ax[1,1].set_title("Output")