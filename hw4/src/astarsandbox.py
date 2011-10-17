import numpy as np
import sys, pygame, math, heapq, time
from pygame.locals import *

from costmap import Costmap
from planner import Astar
from voronoi import Voronoi

heuristic = "crow"
    
def showBoard(screen, board):
    screen.blit(board, (0, 0))
    pygame.display.flip()

def initializePlanner(board, cm, planner="astar", heuristic = None):
    if planner is "astar":
        planner = Astar(heuristic,cm.start,cm.goal,cm)
    #===========================================================================
    # if planner is "voronoi":
    #    planner = Voronoi(cm.start,cm.goal,cm,resolution = 0.1)
    # if planner is "brushfire":
    #    planner = Brushfire(cm.start,cm.goal,cm)
    #===========================================================================
    planner.setPygame(board, cm.cell_size)
    planner.drawStart()
    planner.drawGoal()
    planner.startFinding()
    return planner

if __name__ == "__main__":
    pygame.font.init()
    pygame.init()
    
    resolution = 0.1
    cell_size = 5
    mode = "astar"
    
    (cm, board, screen) = Costmap.homeworkFourMap(resolution, cell_size)
    
    planner = initializePlanner(board,cm, planner = mode, heuristic = "crow")
    showBoard(screen,board)
    
    isFinished = False
    step = False
    reset = False
    showWeights = False
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

        if one:
            heuristic = "crow"
            reset = True
            time.sleep(0.1)
        if two:
            heuristic = "manhattan"
            reset = True
            time.sleep(0.1)
        if three:
            heuristic = "naive"
            reset = True
            time.sleep(0.1)
        if s:
            reset = True
            step = not step
            time.sleep(0.1)
            
        if w:
            reset = True
            showWeights = not showWeights
            time.sleep(0.1)
        if f:
            fast = not fast
            time.sleep(0.1)
        if enter:
            if step is True and isFinished is not True:
                isFinished = planner.iterate()
                showBoard(screen,board)
                time.sleep(0.1)
        if reset:
            del cm
            cm,board,_ = Costmap.homeworkFourMap(resolution, cell_size, restartScreen=screen)
            showBoard(screen,board)
            planner = initializePlanner(board,cm, planner=mode,heuristic = heuristic)
            print("Starting Planner: Astar")
            print("   Heuristic: %s"%heuristic)
            print("   Show Weights: %s"%showWeights)
            print("   Fast Render: %s"%fast)
            print("   Step Mode: %s"%step)
            planner.showWeights = showWeights
            isFinished = False
            reset = False
            
        if step is False:
            if isFinished is not True:
                isFinished = planner.iterate()
                if not fast:
                    showBoard(screen, board)
                    if cell_size >= 50:
                        time.sleep(0.05)
            else:
                showBoard(screen,board)
