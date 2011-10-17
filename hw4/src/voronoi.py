import numpy as np
import sys, pygame, math, heapq, time
from pygame.locals import *

from costmap import Costmap
from planner import Planner

class Voronoi(Planner):
    def startFinding(self):
        for cell in self.costmap.cells:
            self.costmap.cells[cell].f_score = self.distanceToClosestObstacle(cell)
        print len(self.costmap.obstacles)
        self.draw_gradient()
        
    def iterate(self):
        return True
    
    def distanceToClosestObstacle(self,coord):
        min_dist = 1e10
        for obstacle in self.costmap.obstacles:
            new_dist = self.dist(coord,obstacle)
            if new_dist < min_dist:
                min_dist = new_dist
        return min_dist
    
    def dist(self,coord1,coord2):
        return np.sqrt((coord1[0] - coord2[0])**2 + (coord1[1]-coord2[1])**2)
    
    def draw_gradient(self):
        for i in range(0,self.costmap.width_sz):
            for j in range(0,self.costmap.height_sz):
                cell = self.costmap.cells[(i,j)]
                if cell.state != "start" and cell.state != "goal" and cell.state != "Wall":
                    v = int(cell.f_score)
                    self.drawCell(i, j, (5*v,20*v,10*v))


def showBoard(screen, board):
    screen.blit(board, (0, 0))
    pygame.display.flip()

def initializePlanner(board, cm):
    planner = Voronoi(cm.start,cm.goal,cm)
    planner.setPygame(board, cm.cell_size)
    planner.showWeights = True
    planner.drawStart()
    planner.drawGoal()
    planner.startFinding()
    return planner

if __name__ == "__main__":
    pygame.font.init()
    pygame.init()
    
    resolution = 1
    cell_size = 50
    
    (cm, board, screen) = Costmap.homeworkFourMap(resolution, cell_size)
    
    planner = initializePlanner(board,cm)
    showBoard(screen,board)
    
    isFinished = False
    step = False
    reset = False
    showWeights = True
    fast = False
    
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
            key = pygame.key.get_pressed()
            escape = key[pygame.K_ESCAPE]
            enter = key[pygame.K_RETURN]
            delete = key[pygame.K_BACKSPACE]
            right_arrow = key[pygame.K_RIGHT]
            one = key[pygame.K_1]
            two = key[pygame.K_2]
            three = key[pygame.K_3]
            w = key[pygame.K_w]
            s = key[pygame.K_s]
            f = key[pygame.K_f]
            
        if w:
            planner.showWeights = not planner.showWeights
            planner.draw_gradient()
            time.sleep(0.2)
            showBoard(screen, board)
            
        if reset:
            del cm
            cm,board,_ = Costmap.homeworkFourMap(resolution, cell_size, restartScreen=screen)
            showBoard(screen,board)
            planner = initializePlanner(board,cm)
            planner.showWeights = showWeights
            isFinished = False
            reset = False
            
        if step is False:
            if isFinished is not True:
                isFinished = planner.iterate()
                if not fast:
                    showBoard(screen, board)
                    time.sleep(0.01)
