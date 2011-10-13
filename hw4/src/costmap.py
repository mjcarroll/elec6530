import pygame

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
    COLORS = {'wall':(0,0,0),
              'gray' :(112,128,144),
              'start':(0,204,102),
              'goal':(255,44,0)}
    
    def __init__(self,
                 width,
                 height,
                 resolution,
                 cell_size):
        self.width = width
        self.width_sz = int(width * 1 / resolution + 2)
        self.height = height
        self.height_sz = int(height * 1 / resolution + 2)
        self.resolution = resolution
        self.cell_size = cell_size
        self.cells = {}
        
        for x in range(self.width_sz):
            for y in range(self.height_sz):
                self.cells[(x, y)] = Cell(x, y)
    
    @classmethod
    def homeworkFourMap(cls,resolution=1, cell_size = 50, restartScreen = None):
        costmap = Costmap(20, 10, resolution, cell_size)
        
        costmap.start = (1,1)
        costmap.goal = (20*1/resolution, 10*1/resolution)
        
        size = costmap.width_sz * cell_size, costmap.height_sz * cell_size
        if restartScreen is None:
            screen = pygame.display.set_mode(size)
            pygame.display.set_caption('ELEC6530 Homework 4')
        else:
            screen = restartScreen
        
        background = pygame.Surface(screen.get_size())
        background = background.convert()
        background.fill(costmap.COLORS['gray'])
        width = costmap.width_sz
        height = costmap.height_sz
        for i in range(0, (cell_size * width) + 1)[::cell_size]:
            pygame.draw.line(background, costmap.COLORS['wall'], 
                             (i, 0), (i, cell_size * height), 2)
        for i in range(0, (cell_size * height) + 1)[::cell_size]:
            pygame.draw.line(background, costmap.COLORS['wall'], 
                             (0, i), (cell_size * width, i), 2)  
        board = background
        
        costmap.addBorders(board)
        sz = int(3*1/costmap.resolution)
        m = int(1/resolution)
        costmap.addObstacle(board, 2*m, 2*m, sz, sz)
        costmap.addObstacle(board, 8*m, 4*m, sz, sz)
        costmap.addObstacle(board, 15*m, 3*m, sz, sz)
        
        return costmap, board, screen
        
    def addBorders(self, board):
        for i in range(0, self.width_sz):
            self.cells[(i, 0)].state = 'Wall'
            self.cells[(i, self.height_sz - 1)].state = 'Wall'
            self.draw_cell(board, i, 0, self.cell_size, self.COLORS['wall'])
            self.draw_cell(board, i, self.height_sz - 1, self.cell_size, self.COLORS['wall'])
        for i in range(0, self.height_sz):
            self.cells[(0, i)].state = 'Wall'
            self.cells[(self.width_sz - 1, i)].state = 'Wall'
            self.draw_cell(board, 0, i, self.cell_size, self.COLORS['wall'])
            self.draw_cell(board, self.width_sz - 1, i, self.cell_size, self.COLORS['wall'])
        
    def addObstacle(self,board, x, y, w, h):
        for i in range(x + 1, x + w + 1):
            for j in range(y + 1, y + h + 1):
                self.cells[(i, j)].state = 'Wall'
                self.draw_cell(board, i, j, self.cell_size, self.COLORS['wall'])
    
    def draw_cell(self, board, xIndex, yIndex, size, color):
        r = pygame.Rect(xIndex * size + 2, yIndex * size + 2, size - 2, size - 2)
        pygame.draw.rect(board, color, r, 0)
    
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