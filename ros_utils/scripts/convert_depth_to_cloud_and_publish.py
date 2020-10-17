#!/usr/bin/env python

import cv2
import message_filters
import numpy as np
import os
import rospy
import shutil

from glob import glob
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, DisparityImage, PointCloud2, PointField
from std_msgs.msg import Header


def callback_listener(depth_msg):
    global encoding
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding=encoding)
    
    publish(cloud)
    
    
def publisher():
    rospy.init_node('convert_depth_to_cloud_and_publish', anonymous=True)
    depth_topic = rospy.get_param('~left_topic', '/stereo/esgm')
    cloud_publisher = rospy.Publisher('/stereo/left/image_rect', '/stereo/right/image_rect', '/stereo/left/camera_info', '/stereo/right/camera_info', encoding, queue)

    
def listener():
    global encoding
    
    encoding = rospy.get_param('~encoding', '32FC1')
    rospy.init_node('convert_depth_to_cloud_and_publish', anonymous=True)
    depth_topic = rospy.get_param('~depth_topic', '/stereo/esgm')
    rospy.Subscriber(depth_topic, Image, callback_listener)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
