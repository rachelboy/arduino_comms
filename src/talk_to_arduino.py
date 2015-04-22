#!/usr/bin/env python

import rospy
import light_control
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from time import time

robot_state = "stopped"

def stopped_lights():
    pass

def forward_lights():
    pass

def reverse_lights():
    pass

def turn(dir):
    pass

def makeGoLights(data):
    speed = data.linear.x
    angle = data.angular.z
    if angle < -.2:
        turn('right')
    elif angle > .2:
        turn('left')
    if speed == 0:
        stopped_lights()
    elif speed < 0:
        reverse_lights()
    else:
        forward_lights()

if __name__ == "__main__":

    rospy.init_node("talk_to_arduino")
    port = rospy.get_param("arduino_port", "/dev/ttyACM0")
    ser = serial.Serial(port, 9600, timeout=0)

    lights = light_control.Lights(ser)

    twist_sub = rospy.Subscriber("twist", Twist, makeGoLights)

    estop_pub = rospy.Publisher("estop", Bool)
    estop = -1

    stopped_lights()

    r = rospy.Rate(2)
    while not(rospy.is_shutdown()):
        resp = ser.read(500)
        if 'STOP' in resp:
            estop = time()
        if estop >= 0:
            if time()-estop < 2.5:
                estop_pub.publish(Bool(True))
            else:
                estop = -1
        if estop == -1:
            estop_pub.publish(Bool(False))

        r.sleep();
