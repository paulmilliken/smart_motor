#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from smart_motor.srv import *

import time
import serial
from Smartmotor import *

# Instantiate sm globally so it can be used in callback functions callback and
# handleGetDisplacement:
sm = Smartmotor() 

def callback(data):
    if data.data=='doScan':
        sm.doScan()
    elif data.data=='return':
        sm.returnToStartPosition()
    elif data.data=='home':
        sm.home()
    elif data.data=='prePosition':
        sm.prePosition()
    elif data.data=='all':
        sm.runAll()
    else:
        rospy.logerr('Warning: Command %s unknown', str(data.data))

def handleGetDisplacement(req):
    response = sm.getDisplacement()
    return(response)

def initialiseRosStuff():
    rospy.init_node("smart_motor_driver")
    rospy.set_param('sm/is_scanning', False)
    rospy.Subscriber('sm/cmd', String, callback)
    server = rospy.Service('sm/get_sm_displacement', SmartmotorDisplacement,
                                                        handleGetDisplacement)

def main():
    initialiseRosStuff()
    rospy.spin()

if __name__ == '__main__':
    main()
