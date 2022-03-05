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

    def search(self, arena):
        solution_found = False
        open_nodes = arena.open_nodes
        arena.open_nodes=[]
        closed_nodes=[]
        for current_node in open_nodes:
            # if len(arena.nodes)>0:
            #     for n in arena.nodes:
            #         if(not n==current_node):
            #             arena.nodes.append(current_node)
            # else:
            arena.nodes.append(current_node)
            closed_nodes.append(current_node)
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

                if(arena.isValid(node) and \
                    # not any(node== n.parent for n in arena.open_nodes) and \
                    # not any(node== n.parent for n in open_nodes) and \
                    # not any(node== n.parent.parent for n in open_nodes) and \
                    # not any(node== n.parent.parent for n in arena.open_nodes) and \
                    not any(node== n for n in arena.nodes) and \
                    # not any(node== n for n in closed_nodes) and \
                    # not any(node== n.parent for n in closed_nodes) and \
                    # not any(node== n for n in arena.obstacle_nodes) and \
                    # not any(node== n.parent for n in arena.obstacle_nodes) and \
                    not node==current_node and \
                    not node==current_node.parent and \
                    not any(node==n for n in open_nodes) and \
                    not any(node==n for n in arena.open_nodes) and \
                    True):  
                    # for n in arena.open_nodes+open_nodes+closed_nodes+arena.nodes+self.recently_closed:
                    #     if(node ==n):
                    #         node = n
                    #     if(node ==n.parent):
                    #         node = n.parent
                    # if(not node.parent):
                    #     node.parent=current_node
                    # print("Appending node: ",node.x, node.y)
                    costToCome = current_node.costToCome + math.sqrt(math.pow(x_-current_node.x,2)+math.pow(y_-current_node.y,2))
                    if node.costToCome > costToCome:
                        node.parent=current_node
                        node.costToCome=costToCome
                    arena.open_nodes.append(node) 
            self.recently_closed.extend(closed_nodes)    
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
        # for n in arena.open_nodes:
        #     print(n.x, n.y, n.costToCome) 
        # input()
        # Update MAP - Pygame display
        arena.drawAll()
    arena.drawAll()
    input()
