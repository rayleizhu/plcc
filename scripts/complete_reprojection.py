# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from project_pcd_to_img import get_reprojection_img
import camera_model
import yaml
import argparse
import os
import pandas as pd
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np

parser = argparse.ArgumentParser(description="Reproject point cloud to image")
parser.add_argument('-p', '--pcd-csv', type=str, default='../data/output/pcd_complete.csv',
                    help="The diitscrectory containing point cloud patches.")
parser.add_argument('-m', '--img', type=str, default='../data/output/orig.png',
                    help="original image file.")
parser.add_argument('-e', '--extr', type=str, default='../data/output/calib_result.csv',
                    help='csv file containing lidar-camera extrinsics')
parser.add_argument('-i', '--intr', type=str, default='../data/output/camera_info.yaml',
                    help="The camera parameter file.")
parser.add_argument('-o', '--output', type=str, default='../data/output/complete_reprojection.png',
                    help="Path for output result.")
parser.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                        help='Silent mode')
parser.set_defaults(verbose=True)
args = parser.parse_args()


extr = pd.read_csv(args.extr).to_numpy()[0]
rot = R.from_quat(extr[3:]).as_dcm()
t = extr[0:3]

img = cv2.imread(args.img)

with open(args.intr, 'r') as stream:
        dct = yaml.safe_load(stream)

if dct['model_type'] == 'PINHOLE':
    proj_param = dct['projection_parameters']
    dist_param = dct['distortion_parameters']
    h = dct['image_height']
    w = dct['image_width']
    cam = camera_model.PinholeCam(proj_param, dist_param, (h, w))
elif dct['model_type'] == 'MEI':
    proj_param = dct['projection_parameters']
    dist_param = dct['distortion_parameters']
    mirr_param = dct['mirror_parameters']['xi']
    h = dct['image_height']
    w = dct['image_width']
    cam = camera_model.MEICam(proj_param, dist_param, mirr_param, (h,w))
else:
    raise NotImplementedError('Camera type {} not implemented yet!'.format(dct['camera_type']))


pts_pcd = pd.read_csv(args.pcd_csv).to_numpy()[:, 0:3]
pts_cam = (np.dot(rot, pts_pcd.T) + np.expand_dims(t, 1)).T
reproject_img, _ = get_reprojection_img(img, cam, pts_cam, np.array([255., 0., 0.]))
cv2.imwrite(args.output, reproject_img)