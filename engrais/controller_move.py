#!/usr/bin/env python

from evdev import InputDevice, categorize, ecodes, list_devices
import rospy
import os as os
import sys as sys
from time import sleep

#Type 1 events
BUTTON_T = 307 # Button pressed = 1 / Release = 0
BUTTON_S = 304
BUTTON_C = 306
BUTTON_X = 305

BUTTON_L1 = 308
BUTTON_L2 = 310
BUTTON_L3 = 314

BUTTON_R1 = 309
BUTTON_R2 = 311
BUTTON_R3 = 315

BUTTON_P = 316
BUTTON_SHARE = 312
BUTTON_OPTIONS = 313
BUTTON_CENTRAL = 317


#Type 3 Events
DPAD_HORIZONTAL = 16 # -1 = LEFT / 1 = RIGHT / 0 = RELEASED
DPAD_VERTICAL = 17 # -1 = UP / 1 = DOWN / 0 = RELEASED

ANALOG_L_X = 0 # 0 = LEFT / 255 = RIGHT / 123 = CENTER / DISCRETE
ANALOG_L_Y = 1 # 0 = UP / 255 = DOWN / 123 = CENTER / DISCRETE

ANALOG_R_X = 2 # 0 = LEFT / 255 = RIGHT / 123 = CENTER / DISCRETE
ANALOG_R_Y = 5 # 0 = UP / 255 = DOWN / 123 = CENTER / DISCRETE


#Bluetooth address
DEFAULT_DEVADD = "00:04:4b:a5:db:33"
DEFAULT_DEVNAME = "Wireless Controller"
DEFAULT_TMUX = ""

devname = DEFAULT_DEVNAME
devadd = DEFAULT_DEVADD


def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


def waitForController(gamepad):
    while gamepad == None:

        # we get all the connected devices
        devices = [InputDevice(path) for path in list_devices()]

        # we loop over those devices looking for the wireless controller
        for device in devices:
            if (device.phys == devadd) and (device.name == devname) :
                print("\n<"+str(devname)+"> is connected to <"+str(device.path)+">")
                gamepad = device

        if gamepad == None: # this means the controller was not found, the programm has to stop
            os.system('echo "controller not found - sleeping 5s"')
            sleep(5)

    return gamepad


def buttonActionPressed(event):
    if event.code == BUTTON_T:
        print("Triangle up")
    elif event.code == BUTTON_S:
        print("Square up")
    elif event.code == BUTTON_C:
        print("Circle up")
    elif event.code == BUTTON_X:
        print("Cross up")

    elif event.code == BUTTON_L1:
        print("L1 up")
    elif event.code == BUTTON_L2:
        print("L2 up")
    elif event.code == BUTTON_L3:
        print("L3 up")

    elif event.code == BUTTON_R1:
        print("R1 up")
    elif event.code == BUTTON_R2:
        print("R2 up")
    elif event.code == BUTTON_R3:
        print("R3 up")

    elif event.code == BUTTON_P:
        print("P up")
    elif event.code == BUTTON_SHARE:
        print("Share up")
    elif event.code == BUTTON_OPTIONS:
        print("Options up")
    elif event.code == BUTTON_CENTRAL:
        print("Central up")


def buttonActionReleased(event):
    if event.code == BUTTON_T:
        print("Triangle release")
    elif event.code == BUTTON_S:
        print("Square release")
    elif event.code == BUTTON_C:
        print("Circle release")
    elif event.code == BUTTON_X:
        print("Cross release")

    elif event.code == BUTTON_L1:
        print("L1 release")
    elif event.code == BUTTON_L2:
        print("L2 release")
    elif event.code == BUTTON_L3:
        print("L3 release")

    elif event.code == BUTTON_R1:
        print("R1 release")
    elif event.code == BUTTON_R2:
        print("R2 release")
    elif event.code == BUTTON_R3:
        print("R3 release")

    elif event.code == BUTTON_P:
        print("P release")
    elif event.code == BUTTON_SHARE:
        print("Share release")
    elif event.code == BUTTON_OPTIONS:
        print("Options release")
    elif event.code == BUTTON_CENTRAL:
        print("Central release")

@static_vars(last_l_x = 0.0, last_l_y = 0.0, last_r_x = 0.0, last_r_y = 0.0)
def movementCommand(event):
    l_x = movementCommand.last_l_x
    l_y = movementCommand.last_l_y
    r_x = movementCommand.last_r_x
    r_y = movementCommand.last_r_y
   
    if event.code == DPAD_HORIZONTAL:
        if event.value == -1:
            print("D left pressed") 
        elif event.value == 1:
            print("D right pressed")

    elif event.code == DPAD_VERTICAL:
        if event.value == -1:
            print("D Up pressed") 
        elif event.value == 1:
            print("D Down pressed")


    elif event.code == ANALOG_L_X:
        l_x = int(((event.value - 127)/127.0)*20)/20.0
        
        if l_x - movementCommand.last_l_x != 0:
            print("Analog L (" + str(l_x) + ", " + str(l_y) + ")")

        movementCommand.last_l_x = l_x

    elif event.code == ANALOG_L_Y:
        l_y = -int(((event.value - 127)/127.0)*20)/20.0
        
        if l_y - movementCommand.last_l_y != 0:
            print("Analog L (" + str(l_x) + ", " + str(l_y) + ")")

        movementCommand.last_l_y = l_y



    elif event.code == ANALOG_R_X:
        r_x = int(((event.value - 127)/127.0)*20)/20.0
        
        if r_x - movementCommand.last_r_x != 0:
            print("Analog R (" + str(r_x) + ", " + str(r_y) + ")")

        movementCommand.last_r_x = r_x

    elif event.code == ANALOG_L_Y:
        r_y = -int(((event.value - 127)/127.0)*20)/20.0
        
        if r_y - movementCommand.last_r_y != 0:
            print("Analog R (" + str(r_x) + ", " + str(r_y) + ")")

        movementCommand.last_r_y = r_y 

    return l_x, l_y, r_x, r_y


def main():
    os.system('echo "looking for the wireless controller"')

    gamepad = None

    gamepad = waitForController(gamepad)

    os.system('echo "\nReady to use"')

    for event in gamepad.read_loop():
        #filters by event type
        if (event.type == ecodes.EV_KEY):
            if event.value == 1:
                buttonActionPressed(event)
            elif event.value == 0:
                buttonActionReleased(event)


        elif event.type == ecodes.EV_ABS: 
           movementCommand(event)

if __name__ == '__main__':
    main()    
