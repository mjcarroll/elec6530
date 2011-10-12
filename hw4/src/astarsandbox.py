import numpy as np
import sys, pygame, math, heapq, time
from pygame.locals import *

class Cell(object):
    def __init__(self,
                 x,y, 
                 state = None, 
                 f_score = None, 
                 h_score = None, 
                 g_score = None, 
                 parent = None):
        self.x = x
        self.y = y
        self.coords = (x,y)
        self.state = None
        self.f_score = None
        self.h_score = None
        self.g_score = None
        self.parent = None

class Costmap(object):
    def __init__(self,
                 width,
                 height,
                 resolution):
        self.width = width
        self.width_sz = int(width * 1/resolution + 2)
        self.height = height
        self.height_sz = int(height * 1/resolution + 2)
        self.resolution = resolution
        self.cells = {}
        
        for x in range(self.width_sz):
            for y in range(self.height_sz):
                self.cells[(x,y)] = Cell(x,y)
                
    def onMap(self,node):
        x,y = node
        return x>=0 and x<self.width_sz and y>=0 and y<self.height_sz
    
    def orthogonals(self,coordinates):
        x, y = coordinates
        N = x-1, y
        E = x, y+1
        S = x+1, y
        W = x, y-1
        directions = [N, E, S, W]
        return [x for x in directions if self.onMap(x) and self.cells[x].state !='Wall']
    
    def diagonals(self,coordinates):
        x, y = coordinates
        NE = x-1, y+1
        SE = x+1, y+1
        SW = x+1, y-1
        NW = x-1, y-1
        directions = [NE, SE, SW, NW]
        return [x for x in directions if self.onMap(x) and self.cells[x].state !='Wall']
    
    def blocked_diagonal(self, current, diag):
        x, y = current
    
        N = x-1, y
        E = x, y+1
        S = x+1, y
        W = x, y-1
        NE = x-1, y+1
        SE = x+1, y+1
        SW = x+1, y-1
        NW = x-1, y-1
        
        if diag == NE:
            return self.cells[N].state == 'Wall' or self.cells[E].state == 'Wall'
        elif diag == SE:
            return self.cells[S].state == 'Wall' or self.cells[E].state == 'Wall'
        elif diag == SW:
            return self.cells[S].state == 'Wall' or self.cells[W].state == 'Wall'
        elif diag == NW:
            return self.cells[N].state == 'Wall' or self.cells[W].state == 'Wall'
        else:
            return False # Technically, you've done goofed if you arrive here.

class Planner(object):
    def __init__(self, start, goal, costmap):
        self.costmap = costmap
        self.start = start
        self.goal = goal
        
    def setPygame(self, board, cell_size):
        self.board = board
        self.cell_size = cell_size
        
    def drawCell(self,xIndex,yIndex,color):
        size = self.cell_size
        r = pygame.Rect(xIndex*size+2,yIndex*size+2,size-2,size-2)
        pygame.draw.rect(self.board, color, r, 0)
    
    def drawStart(self):
        self.drawCell(self.start[0], self.start[1],bright_green)
    
    def drawGoal(self):
        self.drawCell(self.goal[0], self.goal[1],red)
    
    def iterate(self):
        pass
    
    def getCell(self,x,y):
        return self.costmap.cells[(x,y)]
    
    def getStartCell(self):
        return self.costmap.cells[self.start]
    
    def getGoalCell(self):
        return self.costmap.cells[self.goal]
        
    
class Astar(Planner):
    def __init__(self,heuristic, start,goal,costmap):
        self.open_list = []
        self.pq_dict = {}
        self.closed_list = {}
        if heuristic is "crow":
            self.heuristic = self.crow
        elif heuristic is "manhattan":
            self.heuristic = self.manhattan
        else:
            self.heuristic = self.naive
        super(Astar,self).__init__(start,goal,costmap)
    
    def calc_h(self,node):
        node.h_score = self.heuristic(self.getGoalCell(),node)
    
    def calc_f(self,node):
        node.f_score = node.g_score + node.h_score
    
    def update_child(self, parent, child, cost_to_travel):
        c = self.costmap.cells[child]
        p = self.costmap.cells[parent]
        c.g_score = p.g_score + cost_to_travel
        c.parent = parent
       
    def startFinding(self):
        if self.start != None and self.goal != None:
            self.getStartCell().g_score = 0
            self.calc_h(self.getStartCell())
            self.calc_f(self.getStartCell())
            
        self.open_list.append(self.getStartCell().f_score)
        self.pq_dict[self.getStartCell().f_score] = [self.start]
        if len(self.open_list) == 0:
            return
        f = heapq.heappop(self.open_list)
        if len(self.pq_dict[f]) > 1:
            node = self.pq_dict[f].pop()
        else:
            node = self.pq_dict.pop(f)[0]
            
        heapq.heapify(self.open_list)
        self.closed_list[node] = True

        if node != self.goal and node != self.start:
            self.drawCell(node[0],node[1], blue)
        
        self.next_node = start

    def processNode(self,coord):
        if coord == self.goal:
            print "Cost %d\n" % self.getGoalCell().g_score
            self.unwind_path(self.getGoalCell().parent)
            return
        l = []
        cells = self.costmap.cells
        node = cells[coord]
        for x in self.costmap.diagonals(coord):
            if x not in self.closed_list:
                if cells[x].g_score == None:
                    self.update_child(coord, x, cost_to_travel=14)
                    l.append(x)
                elif cells[x].g_score > node.g_score + 14:
                    self.update_child(coord, x, cost_to_travel=14)
                    l.append(x)

        for x in self.costmap.orthogonals(coord):
            if x not in self.closed_list:
                if cells[x].g_score == None:
                    self.update_child(coord, x, cost_to_travel=10)
                    l.append(x)
                elif cells[x].g_score > node.g_score + 10:
                    self.update_child(coord, x, cost_to_travel=10)
                    l.append(x)
                    
        for x in l:
            if x != goal:
                self.drawCell(x[0],x[1],orange)
                
            if cells[x].f_score in self.pq_dict:
                if len(self.pq_dict[cells[x].f_score]) > 1:
                    self.pq_dict[cells[x].f_score].remove(x)
                else:
                    self.pq_dict.pop(cells[x].f_score)
                self.open_list.remove(cells[x].f_score)
            
            self.calc_h(cells[x])
            self.calc_f(cells[x])
            
            self.open_list.append(cells[x].f_score)
            if cells[x].f_score in self.pq_dict:
                self.pq_dict[cells[x].f_score].append(x)
            else:
                self.pq_dict[cells[x].f_score] = [x]
                
        heapq.heapify(self.open_list)
        if len(self.open_list) == 0:
            print "No Path"
            return
        f = heapq.heappop(self.open_list)
        if len(self.pq_dict[f]) > 1:
            node = self.pq_dict[f].pop()
        else:
            node = self.pq_dict.pop(f)[0]
            
        heapq.heapify(self.open_list)
        self.closed_list[node] = True
        
        if node != self.goal:
            self.drawCell(node[0], node[1], blue)
        self.next_node = node
    
    def unwind_path(self,coord):
        if self.costmap.cells[coord].parent != None:
            self.drawCell(coord[0], coord[1], white)
            self.unwind_path(self.costmap.cells[coord].parent)

        
    def iterate(self):
        self.processNode(self.next_node)      
            
    @classmethod
    def crow(cls,cell0,cell1):
        return np.sqrt((cell0.x - cell1.x)**2 + (cell0.y - cell1.y)**2)*10
    
    @classmethod
    def manhattan(cls,cell0,cell1):
        return abs(cell0.x - cell1.x) + abs(cell0.y-cell1.y) * 10
    
    @classmethod
    def naive(cls,cell0,cell1):
        return 0
        
# Colors
black = (0,0,0)                 # Wall Cells
gray = (112, 128, 144)          # Default Cells
bright_green = (0, 204, 102)    # Start Cell
red = (255, 44, 0)              # Goal Cell
orange = (255, 175, 0)          # Open Cells
blue = (0, 124, 204)            # Closed Cells
white = (250,250,250)           # Not used, yet

def initBoard(screen,width,height,cell_size):
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill(gray)
    for i in range(0,(cell_size*width)+1)[::cell_size]:
        pygame.draw.line(background, black, (i, 0), (i, cell_size*height), 2)
    for i in range(0,(cell_size*height)+1)[::cell_size]:
        pygame.draw.line(background, black, (0, i), (cell_size*width,i), 2)  
    return background

def draw_cell(board,xIndex,yIndex,size,color):
    r = pygame.Rect(xIndex*size+2,yIndex*size+2,size-2,size-2)
    pygame.draw.rect(board, color, r, 0)
    
def showBoard(screen, board):
    screen.blit(board, (0,0))
    pygame.display.flip()
    
def addBorders(cm,board):
    for i in range(0,cm.width_sz):
        cm.cells[(i,0)].state = 'Wall'
        cm.cells[(i,cm.height_sz-1)].state = 'Wall'
        draw_cell(board,i,0,cell_size,black)
        draw_cell(board,i,cm.height_sz-1,cell_size,black)
    for i in range(0,cm.height_sz):
        cm.cells[(0,i)].state = 'Wall'
        cm.cells[(cm.width_sz-1,i)].state = 'Wall'
        draw_cell(board,0,i,cell_size,black)
        draw_cell(board,cm.width_sz-1,i,cell_size,black)
        
def addObstacle(cm,board,x,y,w,h):
    for i in range(x+1,x+w+1):
        for j in range(y+1,y+h+1):
            cm.cells[(i,j)].state = 'Wall'
            draw_cell(board,i,j,cell_size,black)

if __name__ == "__main__":
    width = 10
    height = 20
    cell_size = 30
    resolution = 1
    pygame.init()
    cm = Costmap(width,height,resolution)
    size = cm.width_sz * cell_size, cm.height_sz * cell_size
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption('ELEC6530 Homework 4 - A*')
    board = initBoard(screen,cm.width_sz,cm.height_sz,cell_size)
    
    addBorders(cm,board)
    addObstacle(cm,board,2,2,3,3)
    addObstacle(cm,board,4,8,3,3)
    addObstacle(cm,board,3,15,3,3)
    
    start = (1,1)
    goal = (10,20)
    astar = Astar("crow",start, goal, cm)
    astar.setPygame(board, cell_size)
    astar.drawStart()
    astar.drawGoal()
    
    showBoard(screen,board)
    astar.startFinding()
    showBoard(screen,board)
    
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
        astar.iterate()
        showBoard(screen,board)