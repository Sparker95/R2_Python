# -*- coding: utf-8 -*-
"""
Created on Wed Dec 24 17:23:57 2025

@author: spark
"""

def ReadFileIntoString(filePath):
    f = open(filePath, "rt")
    
    if (f.closed):
        return ""
    
    # Read whole file into string
    fileParts = []
    
    while (True):
        filePart = f.read(4*1024)
        if (len(filePart) == 0):
            break
        fileParts.append(filePart)
        
    fileContent = "".join(fileParts)
    
    return fileContent