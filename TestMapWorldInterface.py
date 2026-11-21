# -*- coding: utf-8 -*-
"""
Created on Tue May  6 23:36:10 2025

@author: spark
"""

import numpy as np
import Map
import Localizer

map = Map.Map(0.05, 20)

roomSize_w = float(2)

roomMins = map.PosWorldToMap((-roomSize_w, -roomSize_w))
roomMaxs = map.PosWorldToMap((roomSize_w, roomSize_w))

# Fill: occupied, only 1px border
for x in range(roomMins[0]-1, roomMaxs[0]+2, 1):
    for y in range(roomMins[1]-1, roomMaxs[1]+2, 1):
        map.GetData()[x,y] = 0

# Fill: empty
for x in range(roomMins[0], roomMaxs[0]+1, 1):
    for y in range(roomMins[1], roomMaxs[1]+1, 1):
        map.GetData()[x,y] = 255
        
locInterface = Localizer.LocalizerInterfaceMap(map, 4, 128)

scan = locInterface.GetScan((0,0,0))