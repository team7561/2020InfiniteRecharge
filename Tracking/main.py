import pygame
import time 
import Colours
import VR_data
import FRC_Smart_Dashboard
from pygame.locals import *

pygame.init()
fieldNum = 1
drawRobot = True
screen = pygame.display.set_mode([500, 500],RESIZABLE)

def draw_robot(x, y):
    robot = pygame.draw.rect(screen, Colours.blue, (x-2, y-2, int(screen.get_width()/25), int(screen.get_width()/25)))
def convertCoord(coord):
    global screen
    return coord*int(screen.get_width()/500*42)

def draw_field(width, height):
    global fieldNum
    galacticSearchA = [[(convertCoord(0),convertCoord(0)),(convertCoord(1),convertCoord(6))],["c3", "d5", "a6"],[(convertCoord(11),convertCoord(0)),(convertCoord(1),convertCoord(6))]]
    galacticSearchB = [[(0,0),(width/5,height/5)],["b3", "d5", "b7"],[(0.8*width, 0),(width/5,height/2)]]
    barrelRacing = [[(convertCoord(0),convertCoord(2)),(convertCoord(2),convertCoord(2))],["d5", "b8", "d10"],[(convertCoord(0),convertCoord(2)),(convertCoord(2),convertCoord(2))]]
    slalom = [[(convertCoord(0),convertCoord(4)),(convertCoord(2),convertCoord(2))],["d4","d5","d6","d7","d8","d10"],[(convertCoord(0),convertCoord(2)),(convertCoord(2),convertCoord(2))]]
    bounce = [[(convertCoord(0),convertCoord(2)),(convertCoord(2),convertCoord(2))],["a3","a6","a9"],[(convertCoord(10),convertCoord(2)),(convertCoord(2),convertCoord(2))]]
    lightspeed = [[(0,0),(width/5,height/5)],["a6","b4","b6","b7","b9","b11","c9","d4","d6","d7","d8","d9","d10"],[(400, 0),(height/5,width/2)]]
    maps = [galacticSearchA, galacticSearchB, barrelRacing, bounce, slalom, lightspeed]
    field = pygame.draw.rect(screen, Colours.white, (0, 0, width, width/2))
    draw_rect(maps[fieldNum-1][0][0],maps[fieldNum-1][0][1],True)
    draw_rect(maps[fieldNum-1][2][0],maps[fieldNum-1][2][1],False)
    for x in range(1,12):
        for y in range(1,6):
            if (chr(y+96)+str(x)) in maps[fieldNum-1][1]:
                draw_dot(convertCoord(x), convertCoord(y))
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
def draw_odometry_data(gameDisplay, odometry, height, width):
    width_gap = 15
    # Declare all the text that will be on the screen
    text_x = my_font.render('X:' + str(round(odometry.x, 4)), True, Colours.red)
    text_y = my_font.render('Y:' + str(round(odometry.y, 4)), True, Colours.green)
    text_rot = my_font.render('Rotate:' + str(round(odometry.rot, 3)), True, Colours.blue)
    # Get the rectangles around the texboxes
    text_rect_x = text_x.get_rect()
    text_rect_y = text_y.get_rect()
    text_rect_rot = text_rot.get_rect()
    #Move the textbox to the correct location 
    text_rect_x.center = (height, width)
    text_rect_y.center = (height, width + 1 * width_gap)
    text_rect_rot.center = (height, width + 2 * width_gap)
    #Add text to screen 
    gameDisplay.blit(text_x, text_rect_x)
    gameDisplay.blit(text_y, text_rect_y)
    gameDisplay.blit(text_rot, text_rect_rot)
def draw_dot(x, y):
    field = pygame.draw.circle(screen, Colours.black, (x, y), 5)
def draw_rect(coord, size, start):
    colour = Colours.red
    if (start):
        colour = Colours.green
    field = pygame.draw.rect(screen, colour, pygame.Rect(coord, size))
def draw_buttons(height, width):
    global screen
    global running
    global fieldNum
    global drawRobot
    width_gap = 15
    # Declare all the text that will be on the screen
    text_heading = my_font.render('Maps', True, Colours.red)
    text_1 = my_font.render('Galactic Search A', True, Colours.red)
    text_2 = my_font.render('Galactic Search B', True, Colours.red)
    text_3 = my_font.render('Lightspeed', True, Colours.red)
    text_4 = my_font.render('Bounce', True, Colours.red)
    text_5 = my_font.render('Slalom', True, Colours.red)
    if drawRobot:
        text_draw = my_font.render('Draw Robot?', True, Colours.green)
    else:
        text_draw = my_font.render('Draw Robot?', True, Colours.red)
    text_clear = my_font.render('Clear Path', True, Colours.red)

    # Get the rectangles around the texboxes
    text_rect_heading = text_heading.get_rect()
    text_rect_1 = text_1.get_rect()
    text_rect_2 = text_2.get_rect()
    text_rect_3 = text_3.get_rect()
    text_rect_4 = text_3.get_rect()
    text_rect_5 = text_5.get_rect()
    text_rect_draw = text_draw.get_rect()
    text_rect_clear = text_clear.get_rect()
    # Move the textbox to the correct location 
    text_rect_heading.center = (height, width)
    text_rect_1.center = (height, width + 1 * width_gap)
    text_rect_2.center = (height, width + 2 * width_gap)
    text_rect_3.center = (height, width + 3 * width_gap)
    text_rect_4.center = (height, width + 4 * width_gap)
    text_rect_5.center = (height, width + 5 * width_gap)
    text_rect_draw.center = (height, width + 6 * width_gap)
    text_rect_clear.center = (height, width + 7 * width_gap)
    # Add text to screen 
    screen.blit(text_heading, text_rect_heading)
    screen.blit(text_1, text_rect_1)
    screen.blit(text_2, text_rect_2)
    screen.blit(text_3, text_rect_3)
    screen.blit(text_4, text_rect_4)
    screen.blit(text_5, text_rect_5)
    screen.blit(text_draw, text_rect_draw)
    screen.blit(text_clear, text_rect_clear)

    pos = pygame.mouse.get_pos()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.VIDEORESIZE:
            width = event.dict['w']
            height = event.dict['h']
            screen = pygame.display.set_mode((event.w, int((event.w/2)+220)), pygame.RESIZABLE)    
            print(event.dict['w'])
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if text_rect_1.collidepoint(pos):
                fieldNum = 1
                print(1)
            if text_rect_2.collidepoint(pos):
                fieldNum = 2
                print(2)
            if text_rect_3.collidepoint(pos):
                fieldNum = 3
                print(3)
            if text_rect_4.collidepoint(pos):
                fieldNum = 4
                print(4)
            if text_rect_5.collidepoint(pos):
                fieldNum = 5
                print(5)
            if text_rect_draw.collidepoint(pos):
                drawRobot = not drawRobot
            if text_rect_clear.collidepoint(pos):
                pass

tracker1 = VR_data.TrackerData()
my_font = pygame.font.SysFont('Comic Sans', 20)

i = 50
running = True
fieldNum = 1
while running:
    screen.fill(Colours.black)
    x, y, width, height = screen.get_rect()
    odometry = FRC_Smart_Dashboard.get_robot_odometry()
    draw_field(width, height)
    draw_robot(odometry.x, odometry.y)
    draw_tracker_data(screen,tracker1,0.8*width,width/2+20)
    draw_odometry_data(screen,odometry,0.6*width,width/2+20)
    draw_buttons(0.4*width,width/2+20)
    pygame.display.update()

pygame.quit()