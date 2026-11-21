# -*- coding: utf-8 -*-
"""
Created on Fri Mar  6 21:37:54 2026

@author: spark
"""

import Map
import MapUtils
import PathFinding

map = MapUtils.ReadMapFromFile(r"D:\Projects\Robot2\Records\Legacy Encoder\Office 26 Jan 2026\Office1.rmap")

distMap = PathFinding.CalculateDistanceTransform(map.m_data)