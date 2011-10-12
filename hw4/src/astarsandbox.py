import numpy as np
import sys, pygame, math, heapq, time
from pygame.locals import *

width = 20
height = 10
cell_size = 60 #5
resolution = 1 #0.1
heuristic = "crow"


class Cell(object):
    def __init__(self,
                 x, y,
                 state=None,
                 f_score=None,
                 h_score=None,
                 g_score=None,
                 parent=None):
        self.x = x
        self.y = y
        self.coords = (x, y)
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
        self.width_sz = int(width * 1 / resolution + 2)
        self.height = height
        self.height_sz = int(height * 1 / resolution + 2)
        self.resolution = resolution
        self.cells = {}
        
        for x in range(self.width_sz):
            for y in range(self.height_sz):
                self.cells[(x, y)] = Cell(x, y)
                
    def onMap(self, node):
        x, y = node
        return x >= 0 and x < self.width_sz and y >= 0 and y < self.height_sz
    
    def orthogonals(self, coordinates):
        x, y = coordinates
        N = x - 1, y
        E = x, y + 1
        S = x + 1, y
        W = x, y - 1
        directions = [N, E, S, W]
        return [x for x in directions if self.onMap(x) and self.cells[x].state != 'Wall']
    
    def diagonals(self, coordinates):
        x, y = coordinates
        NE = x - 1, y + 1
        SE = x + 1, y + 1
        SW = x + 1, y - 1
        NW = x - 1, y - 1
        directions = [NE, SE, SW, NW]
        return [x for x in directions if self.onMap(x) and self.cells[x].state != 'Wall']
    
    def blocked_diagonal(self, current, diag):
        x, y = current
    
        N = x - 1, y
        E = x, y + 1
        S = x + 1, y
        W = x, y - 1
        NE = x - 1, y + 1
        SE = x + 1, y + 1
        SW = x + 1, y - 1
        NW = x - 1, y - 1
        
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
        self.showWeights = False
        
    def setPygame(self, board, cell_size):
        self.board = board
        self.cell_size = cell_size
        if cell_size == 60:
            self.font = pygame.font.Font(None,16)
        elif cell_size == 50:
            self.font = pygame.font.Font(None,14)
        elif cell_size ==40:
            self.font = pygame.font.Font(None,12)
        self.fontHeight = self.font.get_height()
        
    def drawCell(self, xIndex, yIndex, color):
        size = self.cell_size
        r = pygame.Rect(xIndex * size + 2, yIndex * size + 2, size - 2, size - 2)
        pygame.draw.rect(board, color, r, 0)
        
        if self.showWeights:
            if color is white or color is blue:
                s1 = "H = %2.2f"%self.costmap.cells[(xIndex,yIndex)].h_score
                s2 = "G = %2.2f"%self.costmap.cells[(xIndex,yIndex)].g_score
                s3 = "F = %2.2f"%self.costmap.cells[(xIndex,yIndex)].f_score
                if color is blue:
                    textColor = orange
                elif color is white:
                    textColor = black
                elif color is orange:
                    textColor = blue
                text1 = self.font.render(s1,1,textColor)
                text2 = self.font.render(s2,1,textColor)
                text3 = self.font.render(s3,1,textColor)
                self.board.blit(text1,(r.centerx-size/2+2,r.centery-self.fontHeight))
                self.board.blit(text2,(r.centerx-size/2+2,r.centery))
                self.board.blit(text3,(r.centerx-size/2+2,r.centery+self.fontHeight))

    def drawStart(self):
        self.drawCell(self.start[0], self.start[1], bright_green)
    
    def drawGoal(self):
        self.drawCell(self.goal[0], self.goal[1], red)
    
    def iterate(self):
        pass
    
    def getCell(self, x, y):
        return self.costmap.cells[(x, y)]
    
    def getStartCell(self):
        return self.costmap.cells[self.start]
    
    def getGoalCell(self):
        return self.costmap.cells[self.goal]
        
    
class Astar(Planner):
    def __init__(self, heuristic, start, goal, costmap):
        self.open_list = []
        self.pq_dict = {}
        self.closed_list = {}
        if heuristic is "crow":
            self.heuristic = self.crow
        elif heuristic is "manhattan":
            self.heuristic = self.manhattan
        else:
            self.heuristic = self.naive
        super(Astar, self).__init__(start, goal, costmap)
    
    def calc_h(self, node):
        node.h_score = self.heuristic(self.getGoalCell(), node)
    
    def calc_f(self, node):
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
            self.drawCell(node[0], node[1], blue)
        
        self.next_node = self.start

    def processNode(self, coord):
        if coord == self.goal:
            print "Cost %d\n" % self.getGoalCell().g_score
            self.unwind_path(self.getGoalCell().parent)
            return True
        l = []
        cells = self.costmap.cells
        node = cells[coord]
        for x in self.costmap.diagonals(coord):
            if x not in self.closed_list:
                if not self.costmap.blocked_diagonal(coord,x):
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
            if x != self.goal:
                self.drawCell(x[0], x[1], orange)
                
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
    
    def unwind_path(self, coord):
        if self.costmap.cells[coord].parent != None:
            self.drawCell(coord[0], coord[1], white)
            self.unwind_path(self.costmap.cells[coord].parent)
        
    def iterate(self):
        done = self.processNode(self.next_node)
        return done      
            
    @classmethod
    def crow(cls, cell0, cell1):
        return np.sqrt((cell0.x - cell1.x) ** 2 + (cell0.y - cell1.y) ** 2) * 10
    
    @classmethod
    def manhattan(cls, cell0, cell1):
        return (abs(cell0.x - cell1.x) + abs(cell0.y - cell1.y)) * 10
    
    @classmethod
    def naive(cls, cell0, cell1):
        return 0
        
# Colors
black = (0, 0, 0)                 # Wall Cells
gray = (112, 128, 144)          # Default Cells
bright_green = (0, 204, 102)    # Start Cell
red = (255, 44, 0)              # Goal Cell
orange = (255, 175, 0)          # Open Cells
blue = (0, 124, 204)            # Closed Cells
white = (250, 250, 250)           # Not used, yet

def initBoard(screen, width, height, cell_size):
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill(gray)
    for i in range(0, (cell_size * width) + 1)[::cell_size]:
        pygame.draw.line(background, black, (i, 0), (i, cell_size * height), 2)
    for i in range(0, (cell_size * height) + 1)[::cell_size]:
        pygame.draw.line(background, black, (0, i), (cell_size * width, i), 2)  
    return background

def draw_cell(board, xIndex, yIndex, size, color):
    r = pygame.Rect(xIndex * size + 2, yIndex * size + 2, size - 2, size - 2)
    pygame.draw.rect(board, color, r, 0)
    
def showBoard(screen, board):
    screen.blit(board, (0, 0))
    pygame.display.flip()
    
def addBorders(cm, board):
    for i in range(0, cm.width_sz):
        cm.cells[(i, 0)].state = 'Wall'
        cm.cells[(i, cm.height_sz - 1)].state = 'Wall'
        draw_cell(board, i, 0, cell_size, black)
        draw_cell(board, i, cm.height_sz - 1, cell_size, black)
    for i in range(0, cm.height_sz):
        cm.cells[(0, i)].state = 'Wall'
        cm.cells[(cm.width_sz - 1, i)].state = 'Wall'
        draw_cell(board, 0, i, cell_size, black)
        draw_cell(board, cm.width_sz - 1, i, cell_size, black)
        
def addObstacle(cm, board, x, y, w, h):
    for i in range(x + 1, x + w + 1):
        for j in range(y + 1, y + h + 1):
            cm.cells[(i, j)].state = 'Wall'
            draw_cell(board, i, j, cell_size, black)

def homeworkFour(cm,board,resolution,heuristic="crow"):
    addBorders(cm, board)
    sz = int(3 * 1/resolution)
    m = int(1/resolution)
    addObstacle(cm, board, 2*m, 2*m, sz, sz)
    addObstacle(cm, board, 8*m, 4*m, sz, sz)
    addObstacle(cm, board, 15*m, 3*m, sz, sz)
    
    start = (1, 1)
    goal = (20*1/resolution, 10*1/resolution)
    astar = Astar(heuristic, start, goal, cm)
    astar.setPygame(board, cell_size)
    astar.drawStart()
    astar.drawGoal()
    
    astar.startFinding()
    showBoard(screen, board)
    return astar

if __name__ == "__main__":
    # (60,1) works well, as does (5,0.1)

    pygame.font.init()
    pygame.init()
    
    cm = Costmap(width, height, resolution)
    size = cm.width_sz * cell_size, cm.height_sz * cell_size
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption('ELEC6530 Homework 4 - A*')
    board = initBoard(screen, cm.width_sz, cm.height_sz, cell_size)
    astar = homeworkFour(cm,board,resolution)
    
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
        
        if escape:
            del cm
            cm = Costmap(width, height, resolution)
            board = initBoard(screen, cm.width_sz, cm.height_sz, cell_size)
            astar = homeworkFour(cm,board,resolution,heuristic)
            isFinished = False
            
        if one:
            heuristic = "crow"
            reset = True
            
        if two:
            heuristic = "manhattan"
            reset = True

        if three:
            heuristic = "naive"
            reset = True

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
                isFinished = astar.iterate()
                showBoard(screen,board)
                time.sleep(0.1)
        
        if reset:
            del cm
            cm = Costmap(width, height, resolution)
            board = initBoard(screen, cm.width_sz, cm.height_sz, cell_size)
            astar = homeworkFour(cm,board,resolution,heuristic)
            astar.showWeights = showWeights
            isFinished = False
            reset = False
            
        if step is False:
            if isFinished is not True:
                isFinished = astar.iterate()
                if not fast:
                    showBoard(screen, board)
            else:
                showBoard(screen,board)
