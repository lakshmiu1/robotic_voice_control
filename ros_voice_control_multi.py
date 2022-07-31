#!/usr/bin/env python3

"""This module is a simple demonstration of voice control
for ROS turtlebot using speech_recognition 
"""

import argparse
import roslib
import rospy
import os
import speech_recognition as sr

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs


class ASRControl(object):
    """Simple voice control interface for ROS turtlebot

    Attributes:
        pub: where to send commands (default: '/cmd_vel')

    """
    def __init__(self):
        # initialize ROS

        rospy.init_node('voice_cmd_vel')
        rospy.on_shutdown(self.shutdown)

        rate = rospy.Rate(5)

        self.ns = "tb3_2"

        self.thr1 = 0.8 #Laser scan range threshold
        self.thr2 = 0.8

        self.sub_ = rospy.Subscriber(f"/{self.ns}/scan", LaserScan, self.callback)  # Subscriber object which will listen "LaserScan" type messages
                                                                    # from the "/scan" Topic and call the "callback" function

        # you may need to change publisher destination depending on what you run
        self.pub_ = rospy.Publisher(f"/{self.ns}/cmd_vel", Twist, queue_size=10)

        self.r = sr.Recognizer()
        self.m = sr.Microphone()


        self.cmd_vel = Twist() #make sure to make the robot stop by default
        self.cmd_vel.linear.x=0;
        self.cmd_vel.angular.z=0;

        self.obstacle = False 

        # A mapping from keywords or phrases to commands
        #we consider the following simple commands, which you can extend on your own
        self.commands =         ['stop',
                                'forward',
                                'backward',
                                'left',
                                'right',
                                ]
        with self.m as source: self.r.adjust_for_ambient_noise(source, duration=1)
        while not rospy.is_shutdown():
            print("Say something! Obstacle found {}:".format(self.obstacle))
            with self.m as source: audio = self.r.listen(source)
            print("Got it! Now to recognize it...")
            try:
                command = self.r.recognize_google(audio)
                if str is bytes:
                   print(u"You said {}".format(command).encode("utf-8"))
                else:
                   print("You said {}".format(command))
                if (command in self.commands):
                   if command == 'forward':
                      self.cmd_vel.linear.x = 0.2
                      self.cmd_vel.angular.z = 0.0
                   elif command == 'backward':
                      self.cmd_vel.linear.x = -0.2
                      self.cmd_vel.angular.z = 0.0
                   elif command == 'left':
                      self.cmd_vel.linear.x = 0.0
                      self.cmd_vel.angular.z = 0.5
                   elif command == 'right':
                      self.cmd_vel.linear.x = 0.0
                      self.cmd_vel.angular.z = -0.5
                   elif command == 'stop':
                      self.cmd_vel.linear.x = 0.0
                      self.cmd_vel.angular.z = 0.0 
                   self.pub_.publish(self.cmd_vel)
                   rate.sleep()
            except sr.UnknownValueError:
                print("Oops! Didn't catch that")


    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)

    def callback(self, dt):
        if dt.ranges[0]> self.thr1 and dt.ranges[15]> self.thr2 and dt.ranges[345] > self.thr2: # Checks if there are obstacles in front and
                                                                         # 15 degrees left and right (Try changing the
                                                                         # the angle values as well as the thresholds)
           self.obstacle = False 
        else:
           #self.cmd_vel.linear.x = 0.0 # stop
           #self.cmd_vel.angular.z = 0.0
           #self.pub_.publish(self.cmd_vel)  # publish the move object
           self.obstacle = True 

if __name__ == '__main__':
      ASRControl()
      rospy.spin()
