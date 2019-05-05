#!/usr/bin/python
import rospy
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
"""
 ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc /stereo/left/image_raw:=/stereo/left/usb_cam/image_raw /stereo/left/cameranfo:=/stereo/left/usb_cam/camera_info /stereo/right/image_raw:=/stereo/right/usb_cam/image_raw /stereo/right/camera_info:=/stereo/right/usb_cam/camera_info _approximate_sync:=true
retrieved from: https://github.com/jeremyfix/ros_rgb_pcl/blob/c747426f7373ccbf23cd4c93d548fedfaf4b6adc/test_kinect/nodes/testDisparity.py
"""
def disp_cb(msg, bridge, img_pub):
    # Let us load the disparity image
    try:
        disparities = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)

    # And convert them into depth
    # From the doc of stereo_msgs/DisparityImage
    # For disparity d, the depth from the camera is Z = fT/d.
    depth_image = msg.f * msg.T / disparities
    print(msg.max_disparity)
    print(msg.min_disparity)
    Z_min, Z_max = msg.f * msg.T / msg.max_disparity, msg.f * msg.T / msg.min_disparity
    print(disparities)
    cv2.imshow("depth",disparities)
    cv2.waitKey(1)
    img_pub.publish(bridge.cv2_to_imgmsg(depth_image, "passthrough"))

# Ros initialization
rospy.init_node("test_disparity")
bridge = CvBridge()

image_pub = rospy.Publisher("image_out",Image, queue_size=1)
disparity_sub = rospy.Subscriber("/stereo/disparity", DisparityImage, lambda img: disp_cb(img, bridge, image_pub))

# And the main loop simply triggers the update of the plot
rospy.spin()
