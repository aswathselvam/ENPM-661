import pygame
import sys


if __name__ == "__main__":

    pygame.init() 
    HEIGHT, WIDTH = 250, 400
    pygame.display.set_caption("Dijkstra Algorithm - Path Planning on Point Robot")

    #### Create a canvas on which to display everything ####
    screen = pygame.display.set_mode((WIDTH, HEIGHT))


    #### Create a surface with the same size as the window ####
    background = pygame.Surface((WIDTH, HEIGHT))
    
    pygame.draw.rect(background,(50,255,50),(50,50,5,5))

    # #### Blit the surface onto the canvas ####
    # screen.blit(background,(0,0))

    # #### Update the the display and wait ####
    # pygame.display.flip()


    screen.blit(pygame.transform.flip(background, False, True), dest=(0, 0))
    # pygame.display.flip()
    pygame.display.update()

    selectStart=True
    while(True): # your main loop
        # get all events
        ev = pygame.event.get()

        # proceed events
        for event in ev:

            # handle MOUSEBUTTONUP
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                
                #vertical flip
                # pos = (pos[0], HEIGHT - pos[1])

                if selectStart:
                    x1, y1 = pos
                    # screen.fill((0, 0, 0))

                    #### Populate the surface with Start location ####
                    pygame.draw.rect(screen,(50,50,255),(x1,y1,5,5))
                    background.blit(pygame.transform.flip(background, False, True), dest=(0, 0))

                    # screen.blit(background,(0,0))
                    # pygame.display.flip()
                    pygame.display.update()
                    # pygame.draw.rect(background,(255,0,255),(WIDTH-50,HEIGHT-50,50,50))
                    print("Start location placed at: ", pos)
                    selectStart=False
                else:
                    x2, y2 = pos
                    #### Populate the surface with Goal Location ####
                    # pygame.draw.rect(background,(255,50,50),(x2,y2,5,5))
                    # pygame.display.update()
                    # pygame.draw.rect(background,(255,0,255),(WIDTH-50,HEIGHT-50,50,50))
                    selectStart=True
                    print("Goal location placed at: ", pos)
                
            #Exit if window is closed
            if event.type == pygame.QUIT:
                pygame.quit() 
                sys.exit()
        
        #Update MAP - Pygame display

