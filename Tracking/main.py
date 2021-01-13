import pygame
import time 
import Colours
import VR_data
pygame.init()

def draw_robot(x, y):
    robot = pygame.draw.rect(screen, (0, 0, 255), (x-2, y-2, 20, 20))

def draw_field(fieldnum):
    field = pygame.draw.rect(screen, (255, 255, 255), (0, 0, 500, 250))
    for x in range(1,12):
        for y in range(1,6):
            draw_dot(x*42, y*42)
def draw_modes():
    pass

def draw_tracker_data(gameDisplay, tracker, height, width):
    width_gap = 15
    # Declare all the text that will be on the screen
    text_x = my_font.render('X:' + str(round(tracker.x, 4)), True, Colours.red)
    text_y = my_font.render('Y:' + str(round(tracker.y, 4)), True, Colours.green)
    text_z = my_font.render('Z:' + str(round(tracker.z, 4)), True, Colours.blue)
    text_rot_x = my_font.render('Rotate X:' + str(round(tracker.x_rot, 3)), True, Colours.red)
    text_rot_y = my_font.render('Rotate Y:' + str(round(tracker.y_rot, 3)), True, Colours.green)
    text_rot_z = my_font.render('Rotate Z:' + str(round(tracker.z_rot, 3)), True, Colours.blue)
    text_r = my_font.render('Quat R:' + str(round(tracker.r, 3)), True, Colours.red)
    text_i = my_font.render('Quat I:' + str(round(tracker.i, 3)), True, Colours.green)
    text_j = my_font.render('Quat J:' + str(round(tracker.j, 3)), True, Colours.blue)
    text_k = my_font.render('Quat K:' + str(round(tracker.k, 3)), True, Colours.blue)
    # Get the rectangles around the texboxes
    text_rect_x = text_x.get_rect()
    text_rect_y = text_y.get_rect()
    text_rect_z = text_z.get_rect()
    text_rect_rot_x = text_rot_x.get_rect()
    text_rect_rot_y = text_rot_y.get_rect()
    text_rect_rot_z = text_rot_z.get_rect()
    text_rect_r = text_r.get_rect()
    text_rect_i = text_i.get_rect()
    text_rect_j = text_j.get_rect()
    text_rect_k = text_k.get_rect()
    #Move the textbox to the correct location 
    text_rect_x.center = (height, width)
    text_rect_y.center = (height, width + 1 * width_gap)
    text_rect_z.center = (height, width + 2 * width_gap)
    text_rect_rot_x.center = (height, width + 3 * width_gap)
    text_rect_rot_y.center = (height, width + 4 * width_gap)
    text_rect_rot_z.center = (height, width + 5 * width_gap)
    text_rect_r.center = (height, width + 6 * width_gap)
    text_rect_i.center = (height, width + 7 * width_gap)
    text_rect_j.center = (height, width + 8 * width_gap)
    text_rect_k.center = (height, width + 9 * width_gap)
    #Add text to screen 
    gameDisplay.blit(text_x, text_rect_x)
    gameDisplay.blit(text_y, text_rect_y)
    gameDisplay.blit(text_z, text_rect_z)
    gameDisplay.blit(text_rot_x, text_rect_rot_x)
    gameDisplay.blit(text_rot_y, text_rect_rot_y)
    gameDisplay.blit(text_rot_z, text_rect_rot_z)
    gameDisplay.blit(text_r, text_rect_r)
    gameDisplay.blit(text_i, text_rect_i)
    gameDisplay.blit(text_j, text_rect_j)
    gameDisplay.blit(text_k, text_rect_k)

def draw_dot(x, y):
    field = pygame.draw.circle(screen, (0, 0, 0), (x, y), 5)

tracker1 = VR_data.TrackerData()


screen = pygame.display.set_mode([500, 500])
my_font = pygame.font.SysFont('Comic Sans', 20)

i = 50
running = True
while running:
    screen.fill((255, 255, 0))

    draw_field(1)
    draw_robot(i, i)
    draw_tracker_data(screen,tracker1,400,300)
    
    # Flip the display
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    pygame.display.update()

pygame.quit()


