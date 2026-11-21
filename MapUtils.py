# -*- coding: utf-8 -*-
"""
Created on Wed Dec 24 17:20:54 2025

@author: spark
"""

import matplotlib.pyplot as plt

import FileUtils
import StringWordReader
import Map
import Utils

def ReadMapFromFile(filePath):
    fileContent = FileUtils.ReadFileIntoString(filePath)
    
    r = StringWordReader.StringWordReader(fileContent)
    
    r.AssertWord("MAP")
    
    r.AssertWord("FORMAT")
    r.AssertWord("L2INT")
    r.AssertWord("QUANT")
    quantization = r.ReadInt()
    
    if (quantization != 8):
        raise Exception("Only map with quantization = 8 is supported now!")
        
    r.AssertWord("RES_W")
    res_w = r.ReadFloat()
    
    r.AssertWord("XSIZE")
    xSize = r.ReadInt()
    r.AssertWord("YSIZE")
    ySize = r.ReadInt()
    
    r.AssertWord("XCENTER")
    xCenter = r.ReadInt()
    r.AssertWord("YCENTER")
    yCenter = r.ReadInt()
    
    r.AssertWord("MAP_DATA")
    
    map = Map.Map(xSize, ySize, res_w, allocData = True)
    map.SetCenter(xCenter, yCenter)
    mapData = map.GetData_L2Int()
    
    for y in range(ySize):
        r.AssertWord("ROW_ID")
        _y = r.ReadInt()
        if y != _y:
            raise Exception("Wrong row index!")
        r.AssertWord("ROW_DATA")
        for x in range(xSize):
            cellValue = r.ReadInt()
            mapData[x,y] = cellValue
    
    # Hotfix for distMap, but ideally we should fix it on serialization
    mapData[mapData == 65535] = -1
    
    return map

def PlotMapFromFile(filePath, colorMap = "Greys"):
    fig, ax = plt.subplots()
    map = ReadMapFromFile(filePath)
    Utils.PlotMap(map, ax, colorMap)
    return (fig, ax)