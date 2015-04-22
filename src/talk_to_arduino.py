#!/usr/bin/env python

import rospy
import light_control
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from time import time

class ArduinoComm():

    def __init__(self):
        rospy.init_node("talk_to_arduino")
        self.port = rospy.get_param("arduino_port", "/dev/ttyACM1")
        self.ser = serial.Serial(self.port, 9600, timeout=0)

        self.lights = light_control.Lights(self.ser)

        self.twist_sub = rospy.Subscriber("twist", Twist, self.makeGoLights)

        self.estop_pub = rospy.Publisher("estop", Bool)
        self.estop = -1

        self.robot_state = ["stopped","stopped"]
        self.stopped_lights()

        self.light_map = ["FRT": 1,
                          "FRB": 2,
                          "FLT": 3,
                          "FLB": 4,
                          "BRT": 5,
                          "BRB": 6,
                          "BLT": 7,
                          "BLB": 8]


    def stopped_lights(self):
        for light, num in self.light_map.keys():
            if light[3] == "T":
                self.lights.fade(num, "turquoise", 16)
            else:
                self.lights.solid_color(num, [0,0,0])

    def forward_lights(self):
        for light, num in self.light_map.keys():
            if light[3] == "T":
                if light[1] == "F":
                    self.lights.solid_color(num, "white")
                else:
                    self.lights.solid_color(num, "turquoise")

    def reverse_lights(self):
        for light, num in self.light_map.keys():
            if light[3] == "T":
                if light[1] == "B":
                    self.lights.solid_color(num, "white")
                else:
                    self.lights.solid_color(num, "turquoise")

    def turn(self,direction):
        for light, num in self.light_map.keys():
            if light[3] == "B":
                if light[2] == direction:
                    self.lights.blink(num, "amber", 4)
                else:
                    self.lights.solid_color(num, [0,0,0])


    def makeGoLights(self, data):
        speed = data.linear.x
        angle = data.angular.z

        if angle < -.2 and self.robot_state[0] != "right":
            self.robot_state[0] = "right"
            self.turn("R")
        elif angle > .2  and self.robot_state[0] != "left":
            self.robot_state[0] = "left"
            self.turn("L")
        elif self.robot_state[0] != "stopped":
            self.robot_state[0] = "stopped"
            self.turn("off")

        if speed == 0  and self.robot_state[1] != "stopped":
            self.robot_state[1] = "stopped"
            self.stopped_lights()
        elif speed < 0  and self.robot_state[1] != "reverse":
            self.robot_state[1] = "reverse"
            self.reverse_lights()
        elif self.robot_state[1] != "forward":
            self.robot_state[1] = "forward"
            self.forward_lights()

    def execute(self):
        r = rospy.Rate(4)
        while not(rospy.is_shutdown()):
            resp = self.ser.read(500)
            if 'STOP' in resp:
                self.estop = time()
            if self.estop >= 0:
                if time()-self.estop < 2.5:
                    self.estop_pub.publish(Bool(True))
                else:
                    self.estop = -1
            if self.estop == -1:
                self.estop_pub.publish(Bool(False))

            r.sleep();

if __name__ == "__main__":
    arduino_comms = ArduinoComms()
    arduino_comms.execute()
    

    
