import pygame

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

if len(joysticks) < 1:
    print "No joystick entered. Control via sliders."
elif len(joysticks) > 1:
    print " More than 1 joystick provided. Please connect just 1 joystick."
else:
    

class GamePad():

    def __init__(self):
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

        self.joystick = joysticks[0]


