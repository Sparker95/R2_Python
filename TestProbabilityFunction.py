# -*- coding: utf-8 -*-
"""
Created on Mon Dec 29 13:28:35 2025

@author: spark
"""

import scipy.integrate as integrate
import numpy as np
import matplotlib.pyplot as plt

distStart = float(0)
distEnd = float(10)

distCenter = float(1)
distStdDev = float(0.15)
distFloor = float(0.5)

# c - center
def _func(x, center, stdDev, floor):
    x = float(x)
    center = float(center)
    stdDev = float(stdDev)
    
    # Normal PDF
    normDiff = x - center
    normExp = np.exp(-(normDiff*normDiff)/(2*stdDev*stdDev))
    normPdf = normExp
    
    return floor + (1 - floor) * normPdf

result = integrate.quad(lambda x: _func(x, distCenter, distStdDev, distFloor),
                        distStart, distEnd)

if (True):
    x = np.arange(distStart, distEnd, (distEnd-distStart)/512)
    y = np.zeros(x.shape[0])
    for i in range(x.shape[0]):
        y[i] = _func(x[i], distCenter, distStdDev, distFloor)
        
    fig, ax = plt.subplots()
    ax.grid(True)
    ax.plot(x, y, '.b')

print(result)