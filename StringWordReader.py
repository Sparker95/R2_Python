# -*- coding: utf-8 -*-
"""
Created on Wed Dec 24 17:22:00 2025

@author: spark
"""

class StringWordReader:
    def __init__(self, strData):
        SEPARATORS = " \n"
        strLen = len(strData)
        words = []
        
        startId = int(0)
        while (True):
            # Remove leading characters
            while (startId < strLen and (strData[startId] in SEPARATORS)):
                startId += 1
            if (startId == strLen):
                break
            # Seek until next trailing characters
            endId = startId
            while (endId < strLen and not (strData[endId] in SEPARATORS)):
                endId += 1
            words.append(strData[startId : endId])
            startId = endId
            
        self.m_Words = words
        self.m_ReadId = 0
        self.m_WordCount = len(words)
    
    def End(self):
        return self.m_ReadId == self.m_WordCount

    def ReadString(self):
        if (self.m_ReadId == self.m_WordCount):
            return ""
        id = self.m_ReadId
        self.m_ReadId += 1
        return self.m_Words[id]
    
    def ReadFloat(self):
        word = self.ReadString()
        return float(word)
    
    def ReadInt(self):
        word = self.ReadString()
        return int(word)
    
    def ReadBool(self):
        return bool(self.ReadInt())
    
    def AssertWord(self, s):
        word = self.ReadString()
        if (word != s):
            raise Exception("Unexpected word: {}, expected: {}".format(word, s))