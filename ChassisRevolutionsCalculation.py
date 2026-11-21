# -*- coding: utf-8 -*-
"""
Created on Sun Sep 14 16:26:52 2025

@author: spark
"""

import math

N_motor = 14
N_shaft = 72

N_shaft_encoder_pulses = 20

N_diff_in = 13
N_diff_out = 38

R_wheel = 0.065/2

V_car = float(0.1) # m/s

######################
# V = 2*pi*r*f
F_wheel = V_car / (2.0 * math.pi * R_wheel)
F_shaft = F_wheel * N_diff_out / N_diff_in
F_motor = F_shaft * N_shaft / N_motor
F_shaft_encoder_signal = F_shaft * N_shaft_encoder_pulses

print("Car speed: ", V_car, "m/s")
print("F motor: ", F_motor, "Hz = ", F_motor*60, "RPM")
print("F shaft: ", F_shaft, "Hz")
print("F shaft encoder signal: ", F_shaft_encoder_signal, "Hz")
print("T shaft encoder signal: ", 1000/F_shaft_encoder_signal, "ms")



######################
mPerMainShaftRev = 2 * math.pi * R_wheel * (N_diff_in / N_diff_out)
print("Meters per main shaft revolution: ", mPerMainShaftRev*1000, " mm")