# -*- coding: utf-8 -*-
"""
Created on Sat Apr 19 16:58:58 2025

@author: spark
"""

# Functions to populate world with actual objects

def InitScene_0(world):
    w = 0.5 # Wall thickness
    world.AddRect(0, 0, w, 8)  # 0
    world.AddRect(0, 8-w, 6, 8)
    world.AddRect(6-w, 0, 6, 8)
    world.AddRect(0, 0, 6, w)
    world.AddRect(4.2, 1, 4.5, 1.3) # 4
    world.AddRect(3, 3.3, 3.3, 4.3)
    world.AddRect(1.5, 4.3, 4, 4.3+w)
    world.AddRect(2, 4.3+w, 2+w, 5.6)
    world.AddRect(5.25, 4.3, 6, 4.3+w) #8
    world.AddRect(2.0, 3.5, 2.3, 3.8)
    world.AddRect(2.8, 3.8, 3.0, 4.0)
    world.AddRect(3, 1, 3.3, 1.3)
    
    world.AddCircle(1.5, 1.5, 0.5) # 0
    world.AddCircle(3, 7, 0.25)
    world.AddCircle(5, 2, 0.1)
    world.AddCircle(4.75, 2.5, 0.1)
    