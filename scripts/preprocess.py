from __future__ import print_function
import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def get_img_from_rosbag(bag_file, img_topic, mode='single'):
    """
    args:
        bag_file: path to bag_file
        img_topic: topic containing images
    return:
        a *list* of cv2 img array
    """
    assert mode in ['single', 'all']

    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()

    img_list = []
    for topic, msg, t in bag.read_messages(topics=[img_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_list.append(cv_img)
        if mode == 'single':
            break
    bag.close()

    return img_list

def get_extr_cam_from_rosbag(bag_file, tag_names, mode='cam2tag'):
    assert mode in ['cam2tag', 'tag2cam']
    bag = rosbag.Bag(bag_file, 'r')

    tf_list = []
    for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=['/tf']):
        if msg.transforms[0].child_frame_id in tag_names:
            x = msg.transforms[0].transform.translation.x
            y = msg.transforms[0].transform.translation.y
            z = msg.transforms[0].transform.translation.z
            rx = msg.transforms[0].transform.rotation.x
            ry = msg.transforms[0].transform.rotation.y
            rz = msg.transforms[0].transform.rotation.z
            rw = msg.transforms[0].transform.rotation.w
            t = np.array([x, y, z])
            r_quat = np.array([rx, ry, rz, rw])
            print(t, r_quat)
            tf_list.append((r_quat, t))
            
    return tf_list

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Extract image and transformation from camera to tags from rosbag.")
    parser.add_argument('-i', '--input', type=str, default='../data/input/data.bag',
                        help="Input ROS bag.")
    parser.add_argument('ntag', type=int, 
                        help='Number of tags')
    parser.add_argument('-t', '--img-topic', type=str, default='/zed/zed_node/left_raw/image_raw_color',
                        help="Image topic.")
    parser.add_argument('-m', '--oimg', type=str, default='../data/output/orig.png',
                        help="Path for saving extracted image.")
    parser.add_argument('-f', '--otfm', type=str, default='../data/output/tf_cam_to_tag.csv',
                        help="Path for saving transformation file.")

    args = parser.parse_args()

    #img = get_img_from_rosbag(args.input, args.img_topic, mode='single')
    tag_names = ['tag_{:d}'.format(i) for i in range(args.ntag)]
    tf_list = get_extr_cam_from_rosbag(args.input, tag_names, mode='cam2tag')