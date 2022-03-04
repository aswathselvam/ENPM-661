import pygame
import sys
import time 

class Node:
    """
    Node class contains details of a node like location, connection 
    with nearby nodes, parent nodes, distance taken from start point 
    """
    def __init__(self, x ,y):
        self.x = x
        self.y = y
        self.costToCome = float('inf')
        self.connections = {}
        self.parent = None 
        
    def __lt__(self, other):
        return self.costToCome < other.costToCome

class Arena:
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

        self.start_location = Node(0,0) 
        self.goal_location = Node(self.WIDTH, self.HEIGHT)
        self.selectStart = True

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

    def drawAll(self):
        self.background.fill((0, 0, 0))
        self.drawNode()
        self.drawStartLocation()
        self.drawGoalLocation()
        time.sleep(0.01)

    def drawNode(self):
        
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

