import pygame
import sys
from arena import Arena

if __name__ == "__main__":
    
    arena = Arena()

    selectStart=True
    while(True): # your main loop
        # get all events
        arena.updateEvents()
        
        # Update MAP - Pygame display
        arena.drawAll()
        