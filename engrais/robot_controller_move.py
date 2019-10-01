#!/usr/bin/env python

import rospy
import time
import threading
import math 

from evdev import *
from select import select
from Queue import Queue

import roslib
roslib.load_manifest("engrais")

from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64

class ControllerCom(threading.Thread):

    #Type 1 events
    __BUTTON_T = 307 # Button pressed = 1 / Release = 0
    __BUTTON_S = 304
    __BUTTON_C = 306
    __BUTTON_X = 305

    __BUTTON_L1 = 308
    __BUTTON_L2 = 310
    __BUTTON_L3 = 314

    __BUTTON_R1 = 309
    __BUTTON_R2 = 311
    __BUTTON_R3 = 315

    __BUTTON_P = 316
    __BUTTON_SHARE = 312
    __BUTTON_OPTIONS = 313
    __BUTTON_CENTRAL = 317


    #Type 3 Events
    __DPAD_HORIZONTAL = 16 # -1 = LEFT / 1 = RIGHT / 0 = RELEASED
    __DPAD_VERTICAL = 17 # -1 = UP / 1 = DOWN / 0 = RELEASED

    __ANALOG_L_X = 0 # 0 = LEFT / 255 = RIGHT / 123 = CENTER / DISCRETE
    __ANALOG_L_Y = 1 # 0 = UP / 255 = DOWN / 123 = CENTER / DISCRETE

    __ANALOG_R_X = 2 # 0 = LEFT / 255 = RIGHT / 123 = CENTER / DISCRETE
    __ANALOG_R_Y = 5 # 0 = UP / 255 = DOWN / 123 = CENTER / DISCRETE

    __gamepad = None


    def __init__(self, devadd, devname, pubMode, pubEmergency):

        self.__DEFAULT_DEVADD = devadd
        self.__DEFAULT_DEVNAME = devname

        self.__pubMode = pubMode
        self.__pubEmergency = pubEmergency

        super(ControllerCom, self).__init__()

    def run(self):
        global node_name
        
        self.__waitForController()

        while not rospy.is_shutdown():
            try:
                r,w,x = select([self.__gamepad.fd], [], [], 0.1)
                if r:
                    for event in self.__gamepad.read():
                        if (event.type == ecodes.EV_KEY):
                            if event.value == 1:
                                self.__buttonActionPressed(event)
                            elif event.value == 0:
                                self.__buttonActionReleased(event)

                        elif event.type == ecodes.EV_ABS: 
                           self.__movementCommand(event)

            except IOError:
                rospy.logerr(node_name + ': Shutting Down due to lost communication with the controller')
                rospy.signal_shutdown('Lost Comunication')


    def __waitForController(self):
        global controller_ready

        while self.__gamepad == None and not rospy.is_shutdown():

            # we get all the connected devices
            devices = [InputDevice(path) for path in list_devices()]

            # we loop over those devices looking for the wireless controller
            for device in devices:
                if (device.phys == self.__DEFAULT_DEVADD) and (device.name == self.__DEFAULT_DEVNAME) :
                    rospy.loginfo("<"+str(self.__DEFAULT_DEVNAME)+"> is connected to <"+str(device.path)+">")
                    self.__gamepad = device
                    controller_ready = True

            if self.__gamepad == None: # this means the controller was not found, the programm has to stop
                rospy.loginfo("Not able to connect to the controller, trying again in 1s...")
                rospy.sleep(1)


    def __buttonActionPressed(self, event):
        global mode, emergency, node_name

        if event.code == self.__BUTTON_X:
            mode = "automatic"
            self.__pubMode.publish(mode)

        elif event.code == self.__BUTTON_T or event.code == self.__BUTTON_S or event.code == self.__BUTTON_C:
            mode = "manual"
            self.__pubMode.publish(mode)

        elif event.code == self.__BUTTON_CENTRAL:
            if mode == "manual":
                mode = "automatic"
            else:
                mode = "manual"

            self.__pubMode.publish(mode)

        elif event.code == self.__BUTTON_L1 or event.code == self.__BUTTON_L2 or event.code == self.__BUTTON_L3 or event.code == self.__BUTTON_R1 or event.code == self.__BUTTON_R2 or event.code == self.__BUTTON_R3:
            
            mode = "manual"
            emergency = True

            rospy.logerr(node_name + ': Emergency button pressed')
            
            for i in range(10): 
                self.__pubMode.publish(mode)       
                self.__pubEmergency.publish(Bool(True))
                rospy.sleep(0.01)

            rospy.signal_shutdown('Emergency')
       

    def __buttonActionReleased(self, event):
        global mode

        if event.code == self.__BUTTON_X:
            mode = "manual"
            self.__pubMode.publish(mode)


    def __movementCommand(self, event):
        global x, y

        if event.code == self.__ANALOG_L_X:
            x = int(((event.value - 127)/127.0)*20)/20.0
          
        elif event.code == self.__ANALOG_L_Y:
            y = -int(((event.value - 127)/127.0)*20)/20.0



def main():
    global x, y, mode, emergency, node_name, controller_ready

    x = y = 0.0

    mode = 'manual'

    emergency = controller_ready = False

    rospy.init_node('ps4_controller_node', anonymous=False)

    node_name = rospy.get_name()
 
    if not rospy.has_param(node_name + '/pub_topic_right') or not rospy.has_param(node_name + '/pub_topic_left') or not rospy.has_param(node_name + '/emergency_topic') or not rospy.has_param(node_name + '/desired_velocity') or not rospy.has_param(node_name + '/mode_change_topic') or not rospy.has_param(node_name + '/bluetooth_address') or not rospy.has_param(node_name + '/bluetooth_device_name'):

        rospy.logerr(node_name + ": Missing argument, expected 'pub_topic_right', 'pub_topic_left', 'mode_change_topic', 'bluetooth_address', 'bluetooth_device_name', 'emergency_topic' and 'desired_velocity'")

        exit(1)

    
    pubLeft = rospy.get_param(node_name + '/pub_topic_left')
    pubRight = rospy.get_param(node_name + '/pub_topic_right')
    pubMode = rospy.get_param(node_name + '/mode_change_topic')
    
    bluetoothAddr = rospy.get_param(node_name + '/bluetooth_address')
    bluetoothDeviceName = rospy.get_param(node_name + '/bluetooth_device_name')
    
    max_vel = rospy.get_param(node_name + '/desired_velocity')
    emerTopic = rospy.get_param(node_name + '/emergency_topic')


    pubMode = rospy.Publisher(pubMode, String, queue_size=10)
    pubEmergency = rospy.Publisher(emerTopic, Bool, queue_size=10)

    pubLeftWheel= rospy.Publisher(pubLeft, Float64, queue_size=10)
    pubRightWheel = rospy.Publisher(pubRight, Float64, queue_size=10)


    rate = rospy.Rate(50)

    controller_t = ControllerCom(bluetoothAddr, bluetoothDeviceName, pubMode, pubEmergency)
    controller_t.start()

    rospy.loginfo('Node Running, press CTRL+C to exit')

    while not rospy.is_shutdown():
        if controller_ready and mode != 'automatic' and not emergency:

            r = math.sqrt(x**2 + y**2)
            theta = math.atan2(y, x) * 180.0 / math.pi

            if 0.0 <= theta and theta <= 90.0:
                lControl = r * max_vel
                rControl = r * (theta/45.0 - 1.0) * max_vel

            elif 90.0 <= theta and theta <= 180.0:
                lControl = r * (-theta/45.0 + 3.0) * max_vel
                rControl = r * max_vel

            elif -180.0 <= theta and theta <= -90.0:
                lControl = -r * max_vel
                rControl = -r * (theta/45.0 + 3.0) * max_vel

            elif -90.0 <= theta and theta <= -0.0:
                lControl = -r * (-theta/45.0 - 1.0) * max_vel
                rControl = -r * max_vel

            
            pubLeftWheel.publish(lControl)
            pubRightWheel.publish(rControl)

        if not emergency:
            pubEmergency.publish(Bool(False))

        rate.sleep()

    rospy.loginfo('Shutting down...')
    
    controller_t.join()
    
    rospy.loginfo('Code ended without errors')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
