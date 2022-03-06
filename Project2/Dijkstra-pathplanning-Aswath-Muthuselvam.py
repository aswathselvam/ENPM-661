import pygame
import sys
from arena import Arena
import heapq
from queue import PriorityQueue
import math
import time

class Dijkstra:

    def __init__(self):
        self.directions = [ [-1, -1],[-1, 0], [-1, 1],
                            [0, -1], [0, 0], [0, 1],
                            [1, -1], [1, 0], [1, 1]] 
        self.recently_closed=[]

    def search(self, arena):
        solution_found = False
        open_nodes=arena.open_nodes.copy()
        for _,current_node in open_nodes.items():
            if current_node== arena.goal_location:
                arena.goal_location=current_node
                solution_found = True
            for direction in self.directions:
                x_ = current_node.x + direction[0]
                y_ = current_node.y + direction[1]
                node = Arena.Node(x_, y_)
                if (arena.isCollision(x_,y_)):
                    obstacle_node = arena.obstacle_nodes.get((node.x,node.y))
                    if not obstacle_node:
                        arena.obstacle_nodes[(x_, y_)] = node
                    continue

                if(not arena.isValid(node)):
                    continue
                
                # Skip evaluating open nodes:
                open_node_visited = arena.open_nodes.get((node.x,node.y))
                if open_node_visited:
                    # Skip evaluating open nodes' parents:
                    open_node_parent_visited = arena.open_nodes.get((open_node_visited.parent.x,open_node_visited.parent.y))
                    if open_node_parent_visited:
                        continue
                    continue

                # Skip evaluating visited nodes:
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
    arena = Arena()
    dijkstra = Dijkstra()
    solution_found = False
    
    for i in range(arena.WIDTH):
        for j in range(arena.HEIGHT):
            node = Arena.Node(i, j)
            if (arena.isCollision(i,j)):
                obstacle_node = arena.obstacle_nodes.get((node.x,node.y))
                if not obstacle_node:
                    arena.obstacle_nodes[(i, j)] = node
                continue
    arena.start_time = time.time()
    while(not solution_found): # your main loop
        # get all events
        arena.updateEvents()
        
        #Search Dijsktra
        solution_found, arena = dijkstra.search(arena)

        # Update MAP - Pygame display
        arena.drawAll()
    arena.displayResults()
