#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, Quaternion,TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from hercules_msgs.msg import Drive
import tf
import math
from Motors import Motors
#import RPi.GPIO as GPIO
import time
import tf2_ros


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
        self.ticks_per_rotation = 36
        self.rads_per_tick = 2 * math.pi / self.ticks_per_rotation
        self.meters_per_tick = (2 * math.pi * 0.0425) / self.ticks_per_rotation
        
        self.leftDir = 1
        self.rightDir = 1
        
        self.leftCount = 0
        self.rightCount = 0
        
        self.leftCur = 0
        self.rightCur = 0
        
        self.positionX = 0.0
        self.positionY = 0.0
        self.heading = 0.0
        
        self.motorDriver=Motors(left_trim=0, right_trim=0)

	self.left_sub = rospy.Subscriber("/encoder_left", Empty, self.left_encoder,queue_size=1)
	self.right_sub = rospy.Subscriber("/encoder_right", Empty, self.right_encoder,queue_size=1)

        self.twist_sub = rospy.Subscriber("/cmd_drive", Drive, self.cmd_drive_callback,queue_size=1)
        self.velocity_pub = rospy.Publisher("/hw_feedback", JointState, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        
        self.br = tf2_ros.TransformBroadcaster()
   
    def left_encoder(self, msg):
        self.leftCount += 1

    def right_encoder(self, msg):
        self.rightCount += 1

    def cmd_drive_callback(self,data):
        
        left = data.left
        right = data.right
        
        left = self.constrain(left)
        right = self.constrain(right)
        
        if(left < 0):
            self.leftDir = -1
        else:
            self.leftDir = 1
        
        if(right < 0):
            self.rightDir = -1
        else:
            self.rightDir = 1
            
        if(abs(left) + abs(right) > 1):
            self.motorDriver.left(int(left))
            self.motorDriver.right(int(right))
        else:
            self.motorDriver.stop()
        
        
    def left_encoder_callback(self, channel):
        self.leftCount += 1
    
    def right_encoder_callback(self, channel):
        self.rightCount += 1
    
    def constrain(self, val):
        return min(220, max(-220, val))

    def pubJointState(self,time_diff):
        
        current_time = rospy.Time.now()
        odom = Odometry()
        joint = JointState()
        joint.name = ["front_left_wheel","front_right_wheel","back_left_wheel", "back_right_wheel"]
        
        self.deltaLeft = self.leftCount * self.leftDir
        self.deltaRight = self.rightCount * self.rightDir
        
        # joint position and speed 
        self.leftCur = ((self.leftCur) + self.leftCount * self.leftDir) 
        self.rightCur = ((self.rightCur) + self.rightCount * self.leftDir)
        
        leftRad = self.leftCur * self.rads_per_tick
        rightRad = self.rightCur * self.rads_per_tick     
        
        leftSpeed = self.leftDir * self.leftCount * self.rads_per_tick / time_diff
        rightSpeed = self.rightDir * self.rightCount * self.rads_per_tick / time_diff 
        
        # Odometry calcs
        deltaDistance = ((self.leftCount + self.rightCount) / 2 ) * self.meters_per_tick
        
        dLeft = self.leftCount * self.meters_per_tick * self.leftDir
        dRight = self.rightCount * self.meters_per_tick * self.rightDir
        deltaHeading = ( dRight - dLeft ) / (self.rads_per_tick/2)
  
        deltaX = deltaDistance * math.cos(deltaHeading);
        deltaY = deltaDistance * math.sin(deltaHeading);
        
        joint.position = [leftRad,rightRad,leftRad,rightRad]
        joint.velocity = [leftSpeed,rightSpeed,leftSpeed,rightSpeed]
        
        self.leftCount = 0
        self.rightCount = 0
        self.velocity_pub.publish(joint)

def main():
    rospy.init_node('hercules_motors', anonymous=True)
    robot = HardwareController()
    
    lastTime = 0
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        curTime = time.time()
        
        robot.pubJointState(curTime-lastTime)
        
        lastTime = curTime
        rate.sleep()
        
if __name__ == '__main__':
    main()
