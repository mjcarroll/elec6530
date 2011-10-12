'''
Created on Oct 11, 2011

@author: mjcarroll
'''

import numpy as np
from map import Map

class BrushfireMap(Map):
    def __init__(self, width, height, resolution=1):
        super(BrushfireMap,self).__init__(width, height, resolution)