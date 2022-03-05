from threading import currentThread
import pygame
import sys
import time 
import numpy as np
from queue import PriorityQueue


ORANGE = (130,110,70)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)

class Arena:
    class Node:
        """
        Node class contains details of a node like location, connection 
        with nearby nodes, parent nodes, distance from start point 
        """
        def __init__(self, x ,y, parent=None):
            self.x = x
            self.y = y
            self.costToCome = float('inf')
            self.parent = parent 
            
        def __lt__(self, other):
            return self.costToCome < other.costToCome
        
        def __eq__(self, other):
            if other==None:
                    return False
            return self.x == other.x and self.y == other.y


    def __init__(self):
        pygame.init()
        self.HEIGHT, self.WIDTH = 250, 400
        pygame.display.set_caption("Dijkstra Algorithm - Path Planning on Point Robot")

        #### Create a canvas on which to display everything ####
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))

        #### Create a surface with the same size as the window ####
        # This 'background' surface will have the bottom left as origin.
        # All objects will be drawn in this 'bacground' surface.
        self.background = pygame.Surface((self.WIDTH, self.HEIGHT))

        self.nodes = {}
        self.start_location = self.Node(0,0) 
        self.start_location.costToCome=0
        self.start_location.parent= self.Node(0,0)
        self.open_nodes={}
        self.open_nodes[(self.start_location.x,self.start_location.y)]=self.start_location
        self.obstacle_nodes = []
        self.goal_location = self.Node(self.WIDTH-5,self.HEIGHT-5)
        # self.goal_location = self.Node(50,30)
        self.selectStart = True
        self.obstacles = self.createObstacles()

    def updateEvents(self):

        # proceed events
        for event in pygame.event.get():

            # handle MOUSEBUTTONUP
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()

                # vertical flip - for converting to 'background' surface's coordinate system
                pos = (pos[0], self.HEIGHT - pos[1])

                if self.selectStart:
                    self.start_location.x, self.start_location.y = pos
                    print("Start location placed at: ", pos)
                    self.selectStart = False
                else:
                    self.goal_location.x, self.goal_location.y = pos
                    print("Goal location placed at: ", pos)
                    self.selectStart = True

            # Exit if window is closed
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        if self.isCollision(self.start_location.x, self.start_location.y):
            print("[ERROR] Starting point is inside the obstacle!")
            return 
        if self.isCollision(self.goal_location.x, self.goal_location.y):
            print("[ERROR] Ending point is inside the obstacle!")
            return 
        if(self.isValid(self.start_location)):
            return 
        if(self.isValid(self.goal_location)):
            return

    def isValid(self,node): 
        return (0<=node.x<self.WIDTH) and (0<=node.y<self.HEIGHT) \
        and (0<=node.x<self.WIDTH) and (0<=node.y<self.HEIGHT)

    def drawAll(self):
        self.background.fill((0, 0, 0))
        self.drawNode()
        self.drawStartLocation()
        self.drawGoalLocation()
        self.drawPath()
        # time.sleep(0.01)

    def drawNode(self):
        for _,node in self.nodes.items():
            color = ORANGE
            pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))
        # print("OpenNodes: \n",self.open_nodes)
        for _,node in self.open_nodes.items():
            color = YELLOW
            pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))

        for node in self.obstacle_nodes:
            color = WHITE
            pygame.draw.rect(self.background, color, (node.x, node.y, 1, 1))

        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()
        pass

    def drawStartLocation(self):
        #### Populate the surface with Start location ####
        pygame.draw.rect(self.background, (255, 50, 50), (self.start_location.x, self.start_location.y, 5, 5))
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def drawGoalLocation(self):
        #### Populate the surface with Goal Location ####
        pygame.draw.rect(self.background, (50, 255, 50), (self.goal_location.x, self.goal_location.y, 5, 5))
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()

    def drawPath(self):
        leastnode=next(iter(self.open_nodes))
        for n in self.open_nodes:
            if(n<leastnode):
                leastnode=n
        currentNode = leastnode
        currentNode = self.goal_location
        while(currentNode.parent):
            pygame.draw.rect(self.background, (0,255,0), (currentNode.x, currentNode.y, 1, 1))
            currentNode=currentNode.parent
        self.screen.blit(pygame.transform.flip(self.background, False, True), dest=(0, 0))
        pygame.display.update()


    class Hexagon:
        def __init__(self, x, y, Dx):
            self.type = 'polygon'
            self.cx, self.cy = x, y
            a = Dx/np.sqrt(2) # side length
            DY_2 =  np.sqrt(a**2 - (Dx/2)**2)

            self.p1, self.p4 =(x, y + DY_2), (x, y - DY_2)
            self.p2, self.p3 = (x - Dx/2, y+DY_2/2), (x - Dx/2, y-DY_2/2) 
            self.p5, self.p6 = (x + Dx/2, y-DY_2/2), (x + Dx/2, y+DY_2/2)
            self.points = np.array([self.p1, self.p2, self.p3, self.p4, self.p5, self.p6])
            print(f"Hexagon with corners at {self.points}")

        def isInside(self, x,y):
            m12, m34, m45,  m61 = 0.5,  -0.5, 0.5, -0.5   
            b12, b34, b45,  b61 = 36, 165, -35, 235, 
            b56, b23 = 235, 165        
            
            f1 = (y-m12*x - b12 ) < 0
            f2 = x - b23 > 0  
            f3 = (y-m34*x - b34 ) > 0
            f4 = (y-m45*x - b45 ) > 0
            f5 = x - b56 < 0
            f6 = (y-m61*x - b61 ) <0
            return  f1 and f2 and f3 and f4 and f5 and f6 

    class Circle:
        def __init__(self, x, y, radius):
            self.type = 'circle'
            self.x, self.y = x,y
            self.radius = radius
            print(f"Circle at {x, y} with radius {radius}")

        def isInside(self, x,y):
            return  (x - self.x) **2 + (y- self.y)**2 - self.radius**2 < 0 

    class Polygon:
        def __init__(self, *args):
            self.type = 'polygon'
            self.points = np.array(args)
            print(f"Polygon with corners at {self.points}")

        def isInside(self, x,y):
            # Observed empirically using DESMOS
            m12, m23, m34, m41 = -1.24, -3.2, 0.85, 0.32
            b12, b23, b34, b41 =  230,439, 112, 173
            f1 = (y - m12* x - b12) > 0   
            f2 = (y - m23* x - b23) < 0 
            fmidleft = (y - (-0.1)* x - 189) < 0

            fmidright = (y - (-0.1)* x - 189) >= 0
            f3 = (y - m34* x - b34) > 0  
            f4 = (y - m41* x - b41) < 0 

            return f1 and f2 and fmidleft or fmidright and f4 and f3

    def createObstacles(self):
        circObstacle1 = Arena.Circle(10, 10, 5) 
        hexObstacle = Arena.Hexagon(200, 100, 70)      
        circObstacle = Arena.Circle(self.WIDTH-100, 185, 40) 
        p1, p2, p3, p4 = (36, 185), (105, 100), (105-25, 180), (115, 210)
        polygObstacle = Arena.Polygon(p1, p2, p3, p4)
        return [circObstacle1, circObstacle, hexObstacle, polygObstacle] 
    
    def isCollision(self, x,y):
        states = []
        for obstacle in self.obstacles:
            states.append(obstacle.isInside(x, y)) 
        return sum(states) > 0
