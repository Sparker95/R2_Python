# -*- coding: utf-8 -*-
"""
Created on Sat Nov 29 15:00:11 2025

@author: spark
"""

import numpy as np
import Chassis
import StringWordReader
import FileUtils

STR_MOVE_STATE_STOP = "STOP"
STR_MOVE_STATE_FWD = "FWD"
STR_MOVE_STATE_BWD = "BWD"


class SensorStreamBlock:
    def __init__(self):
        self.m_Timestamp_ms = int(0)
        self.m_Odometer = int(0)
        self.m_MoveState = int(Chassis.MOVE_STATE_STOP)
        self.m_Steer = float(0)
        self.m_ScanPolar = []        
    
def ReadSensorStream(filePath):
    blocks = []
        
    fileContent = FileUtils.ReadFileIntoString(filePath)
    
    reader = StringWordReader.StringWordReader(fileContent)    
    
    while (not reader.End()):
        block = SensorStreamBlock()
        
        # Read header
        reader.AssertWord("SENSOR_DATA_HEADER")
        
        reader.AssertWord("TIME")
        block.m_Timestamp_ms = reader.ReadInt()
        
        reader.AssertWord("ODO")
        block.m_Odometer = reader.ReadInt()
        
        reader.AssertWord("MV_STATE")
        mvStateStr = reader.ReadString()
        if (mvStateStr == "FWD"):
            block.m_MoveState = Chassis.MOVE_STATE_FORWARD
        elif (mvStateStr == "BWD"):
            block.m_MoveState = Chassis.MOVE_STATE_BACKWARD
        elif (mvStateStr == "STOP"):
            block.m_MoveState = Chassis.MOVE_STATE_STOP
        else:
            raise Exception
            
        reader.AssertWord("STEER")
        block.m_Steer = reader.ReadFloat()
        
        # Read scan
        reader.AssertWord("SCAN")
        
        reader.AssertWord("N_SAMPLES")            
        nSamples = reader.ReadInt()
        
        angles = []
        distances = []
        for i in range(nSamples):
            reader.AssertWord("SPL")
            angle = reader.ReadFloat()
            distance = reader.ReadFloat()
            angles.append(angle)
            distances.append(distance)
            
        block.m_ScanPolar = np.array([angles, distances])
        
        blocks.append(block)
    
    return blocks


