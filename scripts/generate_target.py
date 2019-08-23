# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import detect_tags
import cv2
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate calibration target.")
    parser.add_argument('-t', '--type', type=str, default='marker',
                        help="target type, 'marker' or 'board'.")
    parser.add_argument('-n', '--num', type=int, default=3,
                        help='number of targets.')
    parser.add_argument('-d', '--dir', type=str, default='../data/output')
    args = parser.parse_args()

    if args.type == 'marker':
        aruco_dict = detect_tags.get_apriltag_dict()
        for i in range(args.num):
            img	= cv2.aruco.drawMarker(aruco_dict, i, 100)
            path = os.path.join(args.dir, 'Marker_{:d}.png'.format(i))
            cv2.imwrite(path, img)
    elif args.type == 'board':
        for i in range(args.num):
            board, aruco_dict = detect_tags.get_charuco_board_and_dict(i)
            img = board.draw((600, 800))
            path = os.path.join(args.dir, 'ChArucoBoard_{:d}.png'.format(i))
            cv2.imwrite(path, img)
    else:
        raise ValueError('target type {} is not surpported!'.format(args.type))
    