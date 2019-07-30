import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", default='../data/input/data.bag',
                        help="Input ROS bag.")
    parser.add_argument("--output_file", default='../data/output/orig.png', help="Output file path.")
    parser.add_argument("--image_topic", default='/ged32/image/front', help="Image topic.")

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')

        cv2.imwrite(args.output_file, cv_img)
        print "Image has been written to %s" % args.output_file
        break

    bag.close()

    return

if __name__ == '__main__':
    main()
