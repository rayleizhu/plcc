# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function
import os
import cv2

import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import numpy as np

import detect_tags

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

def get_tag_cam_tf_from_rosbag(bag_file, tag_names, mode='tag2cam'):
    """
    args:
        bag_file: path to bag file
        tag_names: a list of tag name
        mode: 'tag2cam' or 'cam2tag'. 'tag2cam' means p_cam = Transform(p_tag; tf)
            for returned tf. Such tf is the original output of Apriltag detection
            package. 'cam2tag' means the reverse tf.
    return:
        a dictionary of transformation where the keys are tag names in tag_names
    """
    assert mode in ['cam2tag', 'tag2cam']
    bag = rosbag.Bag(bag_file, 'r')

    tf_dict = {tag:[] for tag in tag_names}
    for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=['/tf']):
        if msg.transforms[0].child_frame_id in tag_names:
            x = msg.transforms[0].transform.translation.x
            y = msg.transforms[0].transform.translation.y
            z = msg.transforms[0].transform.translation.z
            rx = msg.transforms[0].transform.rotation.x
            ry = msg.transforms[0].transform.rotation.y
            rz = msg.transforms[0].transform.rotation.z
            rw = msg.transforms[0].transform.rotation.w
            if mode == 'tag2cam':
                tf = np.array([x, y, z, rx, ry, rz, rw])
            else:
                rot_mat = R.from_quat(np.array(rx, ry, rz, rw)).as_dcm().T
                t = -np.dot(rot_mat, np.array([x, y, z]))
                r_quat = R.from_dcm(rot_mat).as_quat()
                tf = np.concatenate([t, r_quat], axis=0)
            tf_dict[msg.transforms[0].child_frame_id].append(tf)
    for key in tf_dict:
        if not tf_dict[key]:
            raise ValueError('Transformation for {} not found'.format(key))
            #tf_dict[key] = np.zeros(7)
        else:
            # TODO: ourlier removal?
            tf_dict[key] = np.mean(np.stack(tf_dict[key]), axis=0)

    return tf_dict


def get_tag_cam_tf_from_img(img, tag_names, mode='tag2cam'):
    """
    args:
        bag_file: path to bag file
        tag_names: a list of tag name
        mode: 'tag2cam' or 'cam2tag'. 'tag2cam' means p_cam = Transform(p_tag; tf)
            for returned tf. Such tf is the original output of Apriltag detection
            package. 'cam2tag' means the reverse tf.
    return:
        a dictionary of transformation where the keys are tag names in tag_names
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pass

def get_cam_info_from_rosbag(bag_file, topic):
    """
    args:
    return:
    """
    bag = rosbag.Bag(bag_file, 'r')
    d = {}
    for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=[topic]):
        if msg.distortion_model == 'plumb_bob':
            d['model_type'] = 'PINHOLE'
            d['camera_name'] = msg.header.frame_id
            d['image_width'] = msg.width
            d['image_height'] = msg.height
            K = msg.K
            D = msg.D
            #http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
            d['distortion_parameters']={'k1':D[0],
                                        'k2':D[1],
                                        'p1':D[2],
                                        'p2':D[3]}
            d['projection_parameters']={'fx':K[0],
                                        'fy':K[4],
                                        'cx':K[2],
                                        'cy':K[5]}

        elif camera_type == 'MEI':
            pass
        else:
            raise NotImplementedError('camera type {} not implemented yet'.format(camera_type))
        break
    return d


if __name__ == '__main__':
    import pandas as pd
    import argparse
    import yaml

    parser = argparse.ArgumentParser(description="Extract image and transformation from camera to tags from rosbag.")
    parser.add_argument('ntag', type=int, 
                    help='Number of tags')
    parser.add_argument('-i', '--input', type=str, default='../data/input/data.bag',
                        help="Input ROS bag.")
    parser.add_argument('-t', '--img-topic', type=str, default='/zed/zed_node/left/image_rect_color',
                        help="Image topic.")
    parser.add_argument('-c', '--cam-topic', type=str, default='/zed/zed_node/left/camera_info',
                        help="camera info topic.")
    parser.add_argument('-g', '--tgt-type', type=str, default='board',
                        help="target type, 'marker' for Apriltag or 'board' for ChAruco board.")
    parser.add_argument('--marker-len', type=float, default=0.072,
                        help="side length of single marker, only valid when tgt-type is 'board'.")
    parser.add_argument('--square-len', type=float, default=0.090,
                        help="side length of single square on board, only valid when tgt-type is 'board'.") 
    parser.add_argument('-m', '--oimg', type=str, default='../data/output/orig.png',
                        help="Path for saving extracted image.")
    parser.add_argument('-f', '--otfm', type=str, default='../data/output/tf_tag_to_cam.csv',
                        help="Path for saving transformation file.")
    parser.add_argument('-y', '--ocam', type=str, default='../data/output/camera_info.yaml',
                        help="Path for saving camera info ymal file.")

    parser.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                         help='Silent mode')
    parser.set_defaults(verbose=True)
    args = parser.parse_args()

    # extract image
    img = get_img_from_rosbag(args.input, args.img_topic, mode='single')[0]
    cv2.imwrite(args.oimg, img)

    # extract camera info
    cam_info_dict = get_cam_info_from_rosbag(args.input, args.cam_topic)
    with open(args.ocam, 'w') as stream:
        yaml.safe_dump(cam_info_dict, stream, default_flow_style=False)

    # extract camera-tag transformation
    tf_list = []
    if args.tgt_type == 'marker':
        # TODO: use opencv for single marker detection?
        # TODO: flexible tag selection?
        tag_names = ['tag_{:d}'.format(i) for i in range(args.ntag)]
        tf_dict = get_tag_cam_tf_from_rosbag(args.input, tag_names, mode='tag2cam')
        for tag in tag_names:
            tf_list.append(tf_dict[tag])
    elif args.tgt_type == 'board':
        # TODO: refactor this part
        cam_mat = np.array([[cam_info_dict['projection_parameters']['fx'], 0., cam_info_dict['projection_parameters']['cx']],
                            [0., cam_info_dict['projection_parameters']['fy'], cam_info_dict['projection_parameters']['cy']],
                            [0., 0., 1.]])
        dist = np.array([cam_info_dict['distortion_parameters']['k1'],
                         cam_info_dict['distortion_parameters']['k2'],
                         cam_info_dict['distortion_parameters']['p1'],
                         cam_info_dict['distortion_parameters']['p1']])
        tag_names = ['board_{:d}'.format(i) for i in range(args.ntag)]
        for i in range(args.ntag):
            orig_img = img.copy()
            board, aruco_dict = detect_tags.get_charuco_board_and_dict(i,
                                                                       marker_len=args.marker_len,
                                                                       square_len=args.square_len)
            img_with_axis, tf = detect_tags.get_tag_cam_tf_from_board(orig_img, board, aruco_dict, cam_mat, dist)
            tf_list.append(tf)
            cv2.imwrite('../data/output/board_with_axis_{:d}.png'.format(i), img_with_axis)
    else:
        raise ValueError("Target type '{}' is not surpported".format(args.tgt_type))

    tf_array = np.stack(tf_list)
    df = pd.DataFrame(tf_array, index=tag_names,
                      columns=['t.x', 't.y', 't.z', 'r.x', 'r.y', 'r.z', 'r.w'])
    df.to_csv(args.otfm)
    
    if args.verbose:
        print('Camera info:')
        print(cam_info_dict)
        print('Extracted image and cam-tag transformation from {}.'.format(args.input))
        [print('{}:{}'.format(tag, tf_array[i])) for i, tag in enumerate(tag_names)]
        print('Image saved to {}'.format(args.oimg))
        print('Transformation saved to {}'.format(args.otfm))
        print('Camera info saved to {}'.format(args.ocam))

