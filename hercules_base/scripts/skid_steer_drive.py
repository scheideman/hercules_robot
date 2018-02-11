#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msg.msg import JointState
import tf
import math
import Robot
import RPi.GPIO as GPIO

class HardwareController:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = 0
        self.P = 2
        self.D = 50
        self.dx = 0
        self.dy = 0
        
        # encoder sensor setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(22, GPIO.IN)
        GPIO.setup(18, GPIO.IN)
        GPIO.add_event_detect(22, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(18, GPIO.RISING, callback=self.right_encoder_callback)
        
        self.motorDriver = Robot.Robot(left_trim=0, right_trim=0)

        self.twist_sub = rospy.Subscriber("/cmd_vel_drive", Twist, self.twist_callback)
        self.velocity_pub = rospy.Publisher("/hw_feedback", JointState, queue_size=1)

    def twist_callback(self,data):
        
        self.dx = data.linear.x
        self.dw = data.angular.z
        
    def left_encoder_callback(self, channel):
        pass
    
    def right_encoder_callback(self, channel):
        pass

    



def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hercules_motors', anonymous=True)
    robot = HardwareController()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    main()