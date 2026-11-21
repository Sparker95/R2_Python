import numpy as np
import matplotlib.pyplot as plt

inputs = np.array([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.68])

angles = np.array([0, 6.11, 8.13, 12.49, 14.8, 18.19, 23.89, 28.5])

plt.plot(inputs, angles)
plt.grid()