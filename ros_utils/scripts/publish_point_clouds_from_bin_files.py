from os import path, listdir
import argparse
import struct
from math import sqrt
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.point_cloud2 import create_cloud, PointField, PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from tqdm import tqdm

def to_msg(points):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    header = Header(stamp=rospy.Time(), frame_id='hdl64')
    msg = create_cloud(header, fields, points)
    return msg
def main(args):
    rospy.init_node('bin_label_pub', anonymous=True, disable_signals=True)
    rate = rospy.Rate(7)
    # CLOUD PUB
    cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    # IMAGE PUB
    #cv_bridge = CvBridge()
    #image_pub = rospy.Publisher('/image', Image, queue_size=10)
    pc_file_names = sorted(listdir(args.pc_dir_path))
    # print(pc_file_names)
    for pc_file_name in tqdm(pc_file_names[::]):
        # idx = pc_file_name[:-len('.bin')]
        # idx = pc_file_name[:-len('.npy')]
        # print('IDX: %s' % idx)
        # CLOUD
        # print('PC: %s' % pc_file_name)
        pc_file_path = path.join(args.pc_dir_path, pc_file_name)
        points = np.fromfile(pc_file_path, dtype=np.float32).reshape(-1, 4)
        # points = np.fromfile(pc_file_path, dtype=np.float32).reshape(-1, 3)
        # points = np.load(pc_file_path)[:, :3]
        msg = to_msg(points)
        cloud_pub.publish(msg)
        # IMAGE
        # image = cv2.imread(path.join(args.image_dir_path, '%s.png' % idx))
        # image_pub.publish(cv_bridge.cv2_to_imgmsg(image))
        rate.sleep()
    rospy.signal_shutdown("all point clouds are published")
def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-pc', '--pc_dir_path', type=str, required=True,
                        help='Path to directory with point cloud files')
    #parser.add_argument('-i', '--image_dir_path', type=str, required=True,
    #                    help='Path to directory with corresponding images')
    return parser
cfg = None
if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    main(args)
