# -*- coding: utf-8 -*-
"""
Created on Sun Apr 20 16:18:54 2025

@author: spark
"""

import numpy as np
import matplotlib.pyplot as plt

import ICP


N = 9
scan0=np.zeros((2,N))
scan0[0,:]=np.arange(0,N,1)
scan0[1,:]=(1/N)*scan0[0,:]*scan0[0,:]

scan1 = ICP.MRot(0.15) @ scan0
scan1 = scan1 + 0.15

# Test: flip scan1 to ensure correspondance matching works
scan1 = np.fliplr(scan1)

fig,ax = plt.subplots()

ax.plot(scan0[0,:], scan0[1,:], '.r')
ax.plot(scan1[0,:], scan1[1,:], '.b')
ax.grid(True)


ICP.MatchScansCartesian(scan0, scan1, 10)

plt.show()