#!/usr/bin/env python

import numpy as np

class Map(object):
    BORDER = 10
    OBSTACLE = 20
    CLEAR = 0
    
    def __init__(self, width, height, resolution=1):
        """
        Represents a 2D map for a robot.
        width, height, and resolution are in meters
        """
        self.dims = np.array((height, width))
        self.width_sz = width * 1/resolution
        self.height_sz = height * 1/resolution
        self.resolution = resolution
        self.map = np.zeros((self.width_sz + 2, self.height_sz + 2), dtype = np.int)
        
        # Draw borders on the map
        self.map[:,(0,-1)] = self.BORDER
        self.map[(0,-1),:] = self.BORDER
        self.origin = np.array((1,1))
        self.start = np.array((None,None))
        self.goal = np.array((None,None))
        
    def setStart(self,x,y):
        self.start = 1/self.resolution * np.array((x,y))
    
    def setGoal(self,x,y):
        self.goal = 1/self.resolution * np.array((x,y))
    
    def addObstacle(self, pos, dimensions=(3,3)):
        p = np.array(pos)
        d = np.array(dimensions)
        obstacle = 1/self.resolution * np.concatenate((p,p+d))
        self.map[obstacle[0]+1:obstacle[2]+1,obstacle[1]+1:obstacle[3]+1] = self.OBSTACLE

def homework4_map(resolution=1):
    m = Map(20,10,resolution)
    m.setStart(1,1)
    m.setGoal(10,20)
    m.addObstacle((2,2),(3,3))
    m.addObstacle((8,4),(3,3))
    m.addObstacle((15,3),(3,3))
    return m

if __name__ == "__main__":
    m = homework4_map(0.1)
    from matplotlib import pyplot as plt
    plt.imshow(m.map,interpolation='bicubic')
    plt.show()