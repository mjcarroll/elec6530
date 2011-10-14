import pygame, heapq
import numpy as np
import copy

class Planner(object):
    COLORS = {'black':(0,0,0),
              'gray' :(112,128,144),
              'bright_green':(0,204,102),
              'red':(255,44,0),
              'orange':(255,175,0),
              'blue':(0,124,204),
              'white':(250,250,250)}
    
    def __init__(self, start, goal, costmap):
        self.costmap = costmap
        costmap.cells[start].state = 'start'
        costmap.cells[goal].state = 'goal'
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
        else:
            self.font = pygame.font.Font(None,10)
        self.fontHeight = self.font.get_height()
        
    def drawCell(self, xIndex, yIndex, color):
        size = self.cell_size
        if type(color) is str:
            setColor = self.COLORS[color]
        else:
            setColor = color
        r = pygame.Rect(xIndex * size + 2, yIndex * size + 2, size - 2, size - 2)
        pygame.draw.rect(self.board, setColor, r, 0)
        
        if self.showWeights:
            if color is 'white' or color is 'blue':
                s1 = "H = %2.2f"%self.costmap.cells[(xIndex,yIndex)].h_score
                s2 = "G = %2.2f"%self.costmap.cells[(xIndex,yIndex)].g_score
                s3 = "F = %2.2f"%self.costmap.cells[(xIndex,yIndex)].f_score
                if color is 'blue':
                    textColor = self.COLORS['orange']
                elif color is 'white':
                    textColor = self.COLORS['black']
                text1 = self.font.render(s1,1,textColor)
                text2 = self.font.render(s2,1,textColor)
                text3 = self.font.render(s3,1,textColor)
                self.board.blit(text1,(r.centerx-size/2+2,r.centery-self.fontHeight))
                self.board.blit(text2,(r.centerx-size/2+2,r.centery))
                self.board.blit(text3,(r.centerx-size/2+2,r.centery+self.fontHeight))
            elif self.costmap.cells[(xIndex,yIndex)].f_score is not None:
                s = "%2.0f"%self.costmap.cells[(xIndex,yIndex)].f_score
                text = self.font.render(s,1,self.COLORS['white'])
                self.board.blit(text,(r.centerx-size/2+2,r.centery))

    def drawStart(self):
        self.drawCell(self.start[0], self.start[1], self.COLORS['bright_green'])
    
    def drawGoal(self):
        self.drawCell(self.goal[0], self.goal[1], self.COLORS['red'])
    
    def getCell(self, x, y):
        return self.costmap.cells[(x, y)]
    
    def getStartCell(self):
        return self.costmap.cells[self.start]
    
    def getGoalCell(self):
        return self.costmap.cells[self.goal]
    
    def startFinding(self):
        pass
    
    def iterate(self):
        return True
    
    
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
            self.drawCell(node[0], node[1], 'blue')
        
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
                self.drawCell(x[0], x[1], 'orange')
                
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
            self.drawCell(node[0], node[1], 'blue')
        self.next_node = node
    
    def unwind_path(self, coord):
        if self.costmap.cells[coord].parent != None:
            self.drawCell(coord[0], coord[1], 'white')
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
        
