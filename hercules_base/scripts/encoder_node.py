#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, Quaternion,TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import tf
import math
from Motors import Motors
#import RPi.GPIO as GPIO
import time
import tf2_ros
from twisted.internet import reactor
from sysfs.gpio import Controller, OUTPUT, INPUT, RISING

Controller.available_pins = [186,187]

class ReadEncoders:

    def __init__(self):
	
	pin_left_encoder = Controller.alloc_pin(186, INPUT, self.left_encoder, RISING)
	pin_right_encoder = Controller.alloc_pin(187, INPUT, self.right_encoder, RISING)
	
	pin_left_encoder.read()  # Reads pin logic level
	pin_right_encoder.read()

        self.left_pub = rospy.Publisher("/encoder_left", Empty, queue_size=1)
        self.right_pub = rospy.Publisher("/encoder_right", Empty, queue_size=1)
        
        #print(dir(Controller))
        rospy.on_shutdown(self.reactor_stop)

        try:
           reactor.run()
        except:
           pass
    def reactor_stop(self):
        reactor.stop()
   
    def left_encoder(self, number, state):
	self.left_pub.publish(Empty())        

    def right_encoder(self, number, state):
        self.right_pub.publish(Empty())


def main():
    rospy.init_node('encoder_node')
    robot = ReadEncoders()
    

if __name__ == '__main__':
    main()
