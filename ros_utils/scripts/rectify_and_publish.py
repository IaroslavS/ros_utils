#!/usr/bin/env python

from glob import glob
import numpy as np
import cv2
import os
import rospy
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import shutil
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header
import yaml

class StereoRectifier:
    #----------------------------------Initialization-------------------------------------
    def __init__(self, config_filename):
        with open(config_filename, 'r') as f:
            config_cameras = yaml.load(f, Loader=yaml.FullLoader)
        
        fx = config_cameras['left']['intrinsics'][0]
        fy = config_cameras['left']['intrinsics'][1]
        cx = config_cameras['left']['intrinsics'][2]
        cy = config_cameras['left']['intrinsics'][3]
        self.matrix_intrinsics_left = np.array([[fx, 0, cx],
                                                [0, fy, cy],
                                                [0,0,1]])
        fx = config_cameras['right']['intrinsics'][0]
        fy = config_cameras['right']['intrinsics'][1]
        cx = config_cameras['right']['intrinsics'][2]
        cy = config_cameras['right']['intrinsics'][3]
        self.matrix_intrinsics_right = np.array([[fx, 0, cx],
                                                 [0, fy, cy],
                                                 [0,0,1]])
        self.distortions_left = np.array(config_cameras['left']['distortion_coeffs'])
        self.distortions_right = np.array(config_cameras['right']['distortion_coeffs'])
        self.new_matrix_intrinsics_left = np.array(config_cameras['left_rect']['P'])
        self.new_matrix_intrinsics_right = np.array(config_cameras['right_rect']['P'])
        self.R_left = np.array(config_cameras['left']['T'])[:3,:3]
        self.R_left = np.linalg.inv(self.R_left)
        self.R_right = np.array(config_cameras['right']['T'])[:3,:3]
        self.R_right = np.linalg.inv(self.R_right)
        self.output_shape = config_cameras['left_rect']['resolution']
    #--------------------------------End of Initialization------------------------------------

    #-------------------------------Procedure of Rectification-------------------------------------------------------------------------------------
    def rectify_left_image(self, image):
        height, width, channels = image.shape    
        mapx1, mapy1 = cv2.initUndistortRectifyMap(self.matrix_intrinsics_left, self.distortions_left, self.R_left, self.new_matrix_intrinsics_left, 
                                                   (width, height), cv2.CV_32F)
        output_image = cv2.remap(image, mapx1, mapy1, interpolation=cv2.INTER_LINEAR)
        output_image = output_image[:self.output_shape[1], :self.output_shape[0]]
        return output_image
    def rectify_right_image(self, image):
        height, width, channels = image.shape    
        mapx1, mapy1 = cv2.initUndistortRectifyMap(self.matrix_intrinsics_right, self.distortions_right, self.R_right, self.new_matrix_intrinsics_right, 
                                                   (width, height), cv2.CV_32F)
        output_image = cv2.remap(image, mapx1, mapy1, interpolation=cv2.INTER_LINEAR)
        output_image = output_image[:self.output_shape[1], :self.output_shape[0]]
        return output_image
    #-------------------------------End of Rectification---------------------------------------------------------------------------------------------

    #--------------------------------Give out matrix P --------------------------------------
    def get_P_left(self):
        return self.new_matrix_intrinsics_left
    def get_P_right(self):
        return self.new_matrix_intrinsics_right
    def get_output_shape(self):
        return self.output_shape
    #----------------------------------------------------------------------------------------
#----------------------------------------End of class StereoRectifier-----------------------------------------------------------------------------------

def cam_info_from_proj_mat(P):
    info = CameraInfo()
    P = np.array(P).reshape(3, 4)
    info.K = P[:, :3].ravel().tolist()
    info.P = P.ravel().tolist()
    return info

#-----------------------------------------Class ImagePublisher -----------------------------------------
class ImagePublisher:
    def __init__(self,
                 filename_config,
                 output_topic_left,
                 output_topic_right,
                 camera_topic_left,
                 camera_topic_right,
                 encoding,
                 queue,
                 output_format):
        self.image_pub_left = rospy.Publisher(output_topic_left, Image, queue_size=queue)
        self.image_pub_right = rospy.Publisher(output_topic_right, Image, queue_size=queue)
        self.camera_pub_left = rospy.Publisher(camera_topic_left, CameraInfo, queue_size=queue)
        self.camera_pub_right = rospy.Publisher(camera_topic_right, CameraInfo, queue_size=queue)
        self.encoding = encoding
        self.output_format = output_format

        self.Rectifier = StereoRectifier(filename_config)
        self.header = Header(frame_id='stereo')
        self.num_frame = 0
        self.left_camera_info = cam_info_from_proj_mat(self.Rectifier.get_P_left())
        self.right_camera_info = cam_info_from_proj_mat(self.Rectifier.get_P_right())

    def cam_info_from_proj_mat(self, P):
        info = CameraInfo()
        P = np.array(P).reshape(3, 4)
        info.K = P[:, :3].ravel().tolist()
        info.P = P.ravel().tolist()
        return info

    def rectify_and_publish(self, left_message, right_message):
        self.header.stamp = left_message.header.stamp
        left_image_raw = bridge.imgmsg_to_cv2(left_message, desired_encoding=self.encoding)
        right_image_raw = bridge.imgmsg_to_cv2(right_message, desired_encoding=self.encoding)

        left_image_rectified = self.Rectifier.rectify_left_image(left_image_raw)
        right_image_rectified = self.Rectifier.rectify_right_image(right_image_raw)
        if self.output_format == 'mono8' and self.encoding == 'bgr8':
            left_image_rectified = cv2.cvtColor(left_image_rectified, cv2.COLOR_BGR2GRAY)
            right_image_rectified = cv2.cvtColor(right_image_rectified, cv2.COLOR_BGR2GRAY)
        output_left_message = bridge.cv2_to_imgmsg(left_image_rectified, self.output_format)
        output_right_message = bridge.cv2_to_imgmsg(right_image_rectified, self.output_format)
        output_left_message.header = self.header
        output_right_message.header = self.header
        
        self.image_pub_left.publish(output_left_message)
        self.image_pub_right.publish(output_right_message)
        
        self.left_camera_info.header = self.header
        self.right_camera_info.header = self.header
        self.left_camera_info.width = self.Rectifier.get_output_shape()[0]
        self.left_camera_info.height = self.Rectifier.get_output_shape()[1]
        self.right_camera_info.width = self.Rectifier.get_output_shape()[0]
        self.right_camera_info.width = self.Rectifier.get_output_shape()[1]
        self.camera_pub_left.publish(self.left_camera_info)
        self.camera_pub_right.publish(self.right_camera_info)
        if output_path != '':
            cv2.imwrite(os.path.join(output_path,'images',str(left_message.header.seq).zfill(8)+'.png'), left_image_rectified)
            timestamps_file = open(os.path.join(output_path,'timestamps.txt'), 'a')
            timestamps_file.write('{:.9f}\n'.format(left_message.header.stamp.secs+left_message.header.stamp.nsecs/1000000000.000))
        
#-----------------------------------------End of class ImagePublisher --------------------------------------

def callback(left_message, right_message):
    image_publisher.rectify_and_publish(left_message, right_message)


def publisher():
    global bridge
    global image_publisher
    global output_path

    rospy.init_node('rectify_and_publish', anonymous=True)
    queue = rospy.get_param('~queue_size', 10)
    frame_id = rospy.get_param('~frame_id', 'stereo')
    left_topic = rospy.get_param('~left_topic', '/stereo/left/image_raw')
    right_topic = rospy.get_param('~right_topic', '/stereo/right/image_raw')
    encoding = rospy.get_param('~encoding', 'bgr8')
    filename_config = rospy.get_param('~file_config')
    output_path = rospy.get_param('~output_path_left_image_rect', '')
    output_format = rospy.get_param('~output_format', 'mono8')

    if output_path != '':
        timestamps_file = open(os.path.join(output_path,'timestamps.txt'), 'w')
        timestamps_file.close()
        if os.path.exists(os.path.join(output_path,'images')):
            shutil.rmtree(os.path.join(output_path,'images'))
            os.makedirs(os.path.join(output_path,'images'))

    image_publisher = ImagePublisher(filename_config, '/stereo/left/image_rect', '/stereo/right/image_rect', \
                                    '/stereo/left/camera_info', '/stereo/right/camera_info', encoding, queue, output_format)
    bridge = CvBridge()

    topic_list = [message_filters.Subscriber(left_topic, Image), message_filters.Subscriber(right_topic, Image)]
    ts = message_filters.ApproximateTimeSynchronizer(topic_list, 10, 0.05,  allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
