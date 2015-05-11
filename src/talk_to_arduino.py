#!/usr/bin/env python

import rospy
import light_control
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from time import time

class ArduinoComms():
    '''gives the arduino light commands, and reports the state of the estop

    subscribes to /cmd_vel (Twist)
    publishes /estop (Bool)'''

    def __init__(self):
        rospy.init_node("talk_to_arduino")
        self.port = rospy.get_param("arduino_port", "/dev/arduino")
        self.ser = serial.Serial(self.port, 9600, timeout=0)

        self.lights = light_control.Lights(self.ser)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.makeGoLights)

        self.estop_pub = rospy.Publisher("estop", Bool)
        self.estop = -1 # variable for recording information about estop

        self.light_map = {"FRT": 0,
                          "FRB": 1,
                          "FLT": 2,
                          "FLB": 3,
                          "BRT": 4,
                          "BRB": 5,
                          "BLT": 6,
                          "BLB": 7}

        self.robot_state = ["stopped","stopped"] # [turning, direction of travel]
        self.last_command = 0
        self.stopped_lights()



    def stopped_lights(self):
        for light, num in self.light_map.items():
            if light[2] == "T":
                # fade all the top light in and out slowly
                self.lights.fade(num, "turquoise", 16)
            else:
                # turn all the bottom lights off
                self.lights.solid_color(num, [0,0,0])

    def forward_lights(self):
        for light, num in self.light_map.items():
            if light[2] == "T":
                if light[0] == "F":
                    # turn the top front lights white
                    self.lights.solid_color(num, "white")
                else:
                    # turn the top back lights turquoise
                    self.lights.solid_color(num, "turquoise")

    def reverse_lights(self):
        for light, num in self.light_map.items():
            if light[2] == "T":
                if light[0] == "B":
                    # turn the top back lights white
                    self.lights.solid_color(num, "white")
                else:
                    # turn the top front lights turquoise
                    self.lights.solid_color(num, "turquoise")

    def turn(self,direction):
        '''direction should be 'L' or 'R' to indicate turning left or right. 
        Any other input will turn the blinkers off'''
        for light, num in self.light_map.items():
            if light[2] == "B":
                if light[1] == direction:
                    '''blink the the bottom lights in the direction of the turn'''
                    self.lights.blink(num, "amber", 4)
                else:
                    '''turn off the bottom lights not in the direction of the turn'''
                    self.lights.solid_color(num, [0,0,0])


    def makeGoLights(self, data):
        '''control the lights to reflect the movement of the robot'''
        speed = data.linear.x
        angle = data.angular.z

        # robot_state is used so that light commands are sent
        # when the light pattern changes
        time_since = time() - self.last_command()


        if angle < -.15 and (self.robot_state[0] != "right" or time_since >.5):
            self.robot_state[0] = "right"
            self.turn("R")
            self.last_command = time()
        elif angle > .15  and (self.robot_state[0] != "left" or time_since >.5):
            self.robot_state[0] = "left"
            self.turn("L")
            self.last_command = time()
        elif (self.robot_state[0] != "stopped" or time_since >.5):
            self.robot_state[0] = "stopped"
            self.turn("off")
            self.last_command = time()

        if speed > 0.05  and (self.robot_state[1] != "forward" or time_since >.5):
            self.robot_state[1] = "forward"
            self.forward_lights()
            self.last_command = time()
        elif speed < 0.05  and (self.robot_state[1] != "reverse" or time_since >.5):
            self.robot_state[1] = "reverse"
            self.reverse_lights()
            self.last_command = time()
        elif (self.robot_state[1] != "stopped" or time_since >.5):
            self.robot_state[1] = "stopped"
            self.stopped_lights()
            self.last_command = time()

    def execute(self):
        r = rospy.Rate(4)
        while not(rospy.is_shutdown()):
            resp = self.ser.read(500) # read messages from arduino
            if 'STOP' in resp: 
                # if it says the estop is engaged, record the time
                self.estop = time() 
            if self.estop >= 0:
                # arduino publishes estop state every 2 seconds
                if time()-self.estop < 2.1:
                    # if estop could still be engaged, publish true
                    self.estop_pub.publish(Bool(True))
                else:
                    # if it's been too long, estop is no longer engaged
                    self.estop = -1
            if self.estop == -1:
                # if estop is not engaged, publish false
                self.estop_pub.publish(Bool(False))

            r.sleep();

if __name__ == "__main__":
    arduino_comms = ArduinoComms()
    arduino_comms.execute()
    

    
