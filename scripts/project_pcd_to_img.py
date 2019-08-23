# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import pandas as pd
import camera_model

def get_reprojection_img(orig_img, camera, pts_cam, color, pt_size=1):
    """
    args:
        orig_img: cv2 image (ndarray)
        camera: Camera object
        pts_cam: (n, 3) ndarray. coordinates of points in camera frame
    return:
        reprojected image
    """
    img = orig_img.copy()
    coord = camera.space_to_plane(pts_cam)
    for i in range(len(coord)):
        if coord[i][0] < camera.width and coord[i][1] < camera.height and \
           coord[i][0] >= 0 and coord[i][1] >= 0:
            cv2.circle(img, (coord[i][0], coord[i][1]), pt_size, color, thickness=-1)
    return img, coord


if __name__ == '__main__':
    import argparse
    import os
    import utility
    import yaml

    parser = argparse.ArgumentParser(description="Reproject point cloud and intersections to image")
    parser.add_argument('-p', '--pcd', type=str, default='../data/output/pcd_patches',
                        help="The diitscrectory containing point cloud patches.")
    parser.add_argument('--itsc-pcd', type=str, default='../data/output/intersection_src.csv',
                        help="csv file containing intersection points in lidar frame.")
    parser.add_argument('--itsc-cam', type=str, default='../data/output/intersection_tgt.csv',
                        help="csv file containing intersection points in camera frame.")
    parser.add_argument('-m', '--img', type=str, default='../data/output/orig.png',
                        help="csv file containing intersection points.")
    parser.add_argument('-t', '--tfm', type=str, default='../data/output/calib_result.csv',
                        help='csv file containing transformation from point cloud to camera')
    parser.add_argument('-c', '--cam', type=str, default='../data/output/camera_info.yaml',
                        help="The camera parameter file.")
    parser.add_argument('-o', '--output', type=str, default='../data/output/reprojection.png',
                        help="Path for output result.")
    parser.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                         help='Silent mode')
    parser.set_defaults(verbose=True)
    args = parser.parse_args()

    extr = pd.read_csv(args.tfm).to_numpy()[0]
    rot = R.from_quat(extr[3:]).as_dcm()
    t = extr[0:3]

    img = cv2.imread(args.img)
    pcd_csv_ls = [ os.path.join(args.pcd, fname) for fname in os.listdir(args.pcd) ]

    # default image channel order is 'bgr' in opencv
    color_ls = utility.get_color_map(len(pcd_csv_ls), mode='bgr')

    with open(args.cam, 'r') as stream:
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
        cam = camera_model.MEICam(proj_param, dist_param, mirr_param, (h, w))
    else:
        raise NotImplementedError('Camera type {} not implemented yet!'.format(dct['camera_type']))

    # project point cloud
    for i, fname in enumerate(pcd_csv_ls):
        pts_pcd = pd.read_csv(fname).to_numpy()
        pts_cam = (np.dot(rot, pts_pcd.T) + np.expand_dims(t, 1)).T
        img, _ = get_reprojection_img(img, cam, pts_cam, np.array(color_ls[i])*255)

    # project plane intersection of lidar frame
    intersec_pcd = pd.read_csv(args.itsc_pcd).to_numpy()
    intersec_pcd_cam = (np.dot(rot, intersec_pcd.T) + np.expand_dims(t, 1)).T
    img, coord_pcd_itsc = get_reprojection_img(img, cam, intersec_pcd_cam, np.array([255, 255, 255]))
    
    # project plane intersection of camera frame
    intersec_cam = pd.read_csv(args.itsc_cam).to_numpy()
    img, coord_cam_itsc = get_reprojection_img(img, cam, intersec_cam, np.array([0, 0, 0]))

    # save reprojected image
    cv2.imwrite(args.output, img)

    if args.verbose:
        print('---------------------------------------------------')
        print('Projection of interscections in pcd frame:{}'.format(coord_pcd_itsc))
        print('Projection of interscections in cam frame:{}'.format(coord_cam_itsc))
        print('Reprojected results saved to {}.'.format(args.output))
