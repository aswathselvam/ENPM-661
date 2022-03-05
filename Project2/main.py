from cvxpy import Solution
import pygame
import sys
from arena import Arena
import heapq
from queue import PriorityQueue
import math

GRAY = (220, 220, 220)
BLUE = (0, 20, 108)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)
class Dijkstra:

    def __init__(self):
        self.directions = [ [-1, -1],[-1, 0], [-1, 1],
                            [0, -1], [0, 0], [0, 1],
                            [1, -1], [1, 0], [1, 1]] 
        self.recently_closed=[]
        self.closed={}           

    def search(self, arena):
        solution_found = False
        open_nodes=arena.open_nodes.copy()
        for nodecoord,current_node in open_nodes.items():
            # if len(arena.nodes)>0:
            #     for n in arena.nodes:
            #         if(not n==current_node):
            #             arena.nodes.append(current_node)
            # else:
            # print(nodecoord)
            # print(current_node.x, current_node.y)
            if current_node== arena.goal_location:
                arena.goal_location=current_node
                solution_found = True
            for direction in self.directions:
                x_ = current_node.x + direction[0]
                y_ = current_node.y + direction[1]
                node = Arena.Node(x_, y_)
                if (arena.isCollision(x_,y_)):
                    if(not any(node==n for n in arena.obstacle_nodes) ):
                        arena.obstacle_nodes.append(node)
                        continue

                if(not arena.isValid(node)):
                    continue

                node_visited = arena.nodes.get((node.x,node.y))
                # print(node_visited,node.x,node.y)
                if node_visited:
                    continue
                
                costToCome = current_node.costToCome + math.sqrt(math.pow(x_-current_node.x,2)+math.pow(y_-current_node.y,2))
                if node.costToCome > costToCome:
                    node.parent=current_node
                    node.costToCome=costToCome
                arena.open_nodes[(node.x,node.y)]=node
            arena.nodes[(current_node.x, current_node.y)] = current_node
            del arena.open_nodes[(current_node.x, current_node.y)]
        return solution_found, arena

if __name__ == "__main__":
    # a = Arena.Node(1,1)
    # b=[Arena.Node(1,1)]
    # c=Arena.Node(1,1)
    # print(any(a==n for n in b))

    import time 
    arena = Arena()
    dijkstra = Dijkstra()
    solution_found = False
    while(not solution_found): # your main loop
        # get all events
        arena.updateEvents()
        
        solution_found, arena = dijkstra.search(arena)
        # for key,val in arena.open_nodes.items():
        #     print(key) 
        # input()
        # Update MAP - Pygame display
        arena.drawAll()
    arena.drawAll()
    input()
