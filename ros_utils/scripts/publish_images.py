#!/usr/bin/env python

from glob import glob
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header
import time


class ImagePublisher:
    def __init__(self,
                 path,
                 topic,
                 encoding,
                 queue,
                 camera_info=None,
                 camera_topic=None):
        self.read_messages(path, encoding)
        self.image_pub = None
        self.camera_pub = None
        if len(self.messages) > 0:
            self.image_pub = rospy.Publisher(topic, Image, queue_size=queue)

            if camera_info is not None and camera_topic is not None:
                self.camera_pub = rospy.Publisher(camera_topic,
                                                  CameraInfo,
                                                  queue_size=queue)
                self.camera_info = camera_info
                self.camera_info.width = self.messages[0].width
                self.camera_info.height = self.messages[0].height

    def read_messages(self, path, encoding):
        print("Reading images from the path" + path)
        self.messages = []
        bridge = CvBridge()
        for fname in sorted(glob(path)):
            img = cv2.imread(fname, cv2.IMREAD_UNCHANGED)
            self.messages.append(bridge.cv2_to_imgmsg(img, encoding))
        print("images in one folder have been read, total number of images - "+str(len(self.messages)))

    def publish(self, header, index):
        if index == len(self.messages):
            self.image_pub = None
        if self.image_pub is not None:
            msg = self.messages[index]
            msg.header = header
            self.image_pub.publish(msg)
            if self.camera_pub is not None:
                self.camera_info.header = header
                self.camera_pub.publish(self.camera_info)

    def __len__(self):
        return len(self.messages)


def cam_info_from_proj_mat(P):
    info = CameraInfo()
    P = np.array(P).reshape(3, 4)
    info.K = P[:, :3].ravel().tolist()
    info.P = P.ravel().tolist()
    return info


def publisher():
    rospy.init_node('image_publisher', anonymous=True, disable_signals=True)
    rate = rospy.get_param('~rate', 5)
    queue = rospy.get_param('~queue_size', 10)
    frame_id = rospy.get_param('~frame_id', 'stereo')
    fname_left = rospy.get_param('~fname_left')
    fname_right = rospy.get_param('~fname_right')
    fname_disp = rospy.get_param('~fname_disp')
    encoding = rospy.get_param('~encoding', 'bgr8')
    pmat_l = rospy.get_param('~pmat_l', [])
    pmat_r = rospy.get_param('~pmat_r', [])

    publishers = []
    publishers.append(
        ImagePublisher(fname_left, 'left/image_rect', encoding, queue,
                       cam_info_from_proj_mat(pmat_l), 'left/camera_info'))
    publishers.append(
        ImagePublisher(fname_right, 'right/image_rect', encoding, queue,
                       cam_info_from_proj_mat(pmat_r), 'right/camera_info'))
    time.sleep(1)
    n = 0
    header = Header(frame_id=frame_id)
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        for pub in publishers:
            pub.publish(header, n)
        n += 1
        if n == len(publishers[0].messages):
            rospy.signal_shutdown("all images are published")
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
