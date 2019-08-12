# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import pandas as pd

class Camera:
    def __init__(self, **kwargs):
        self.model_type = kwargs['model_type']
        assert self.model_type in ['PINHOLE', 'MEI']
        if self.model_type == 'PINHOLE':
            p = kwargs['projection_parameters']
            self.intr = np.array([[p['fx'], 0, p['cx']],
                                  [0, p['fy'], p['cy']],
                                  [0, 0, 1]])
            d = kwargs['distortion_parameters']
            self.dist = (d['k1'], d['k2'], d['p1'], d['p2'])
        #TODO: add 'MEI' model
        else:
            pass

    def project_pts(self, pts):
        """
        args:
            pts: (n, 3) numpy array
        return:
            coord: (n, 2) np array
        """
        if self.model_type == 'PINHOLE':
            pts_cam_front = pts[pts[:, 2] > 1e-3]  # z > 0
            coord_hom = np.dot(self.intr, pts_cam_front.T).T
            coord = coord_hom[:, 0:2] / np.expand_dims(coord_hom[:, 2], 1)
            coord = coord.astype(int)
            # TODO: consider distortion
        elif self.model_type == 'MEI':
            pass
        else:
            raise NotImplementedError("Camera model {} is not implemented!".format(self.model_type))
        return coord


def get_reprojection_img(orig_img, camera, pts_cam, color, pt_size=3):
    """
    args:
        orig_img: cv2 image (ndarray)
        camera: Camera object
        pts_cam: (n, 3) ndarray. coordinates of points in camera frame
    return:
        reprojected image
    """
    coord = camera.project_pts(pts_cam)
    for i in range(len(coord)):
        cv2.circle(img, (coord[i][0], coord[i][1]), pt_size, color, thickness=-1)
    return img


if __name__ == '__main__':
    import argparse
    import os
    import utility
    import yaml

    parser = argparse.ArgumentParser(description="Reproject point cloud and intersections to image")
    parser.add_argument('-p', '--pcd', type=str, default='../data/output/pcd_patches',
                        help="The diitscrectory containing point cloud patches.")
    parser.add_argument('-i', '--itsc', type=str, default='../data/output/intersections_pcd.csv',
                        help="csv file containing intersection points.")
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

    cam = Camera(**dct)

    # project point cloud
    for i, fname in enumerate(pcd_csv_ls):
        pts_pcd = pd.read_csv(fname).to_numpy()
        pts_cam = (np.dot(rot, pts_pcd.T) + np.expand_dims(t, 1)).T
        img = get_reprojection_img(img, cam, pts_cam, np.array(color_ls[i])*255)

    # TODO: proejct intersections
    # # project plane intersection of lidar frame
    # intersec_pcd = pd.read_csv('../data/output/plane_normals_and_intersection_pcd.csv', index_col=0).to_numpy()[-1]
    # intersec_pcd_cam = np.dot(rot, intersec_pcd) + t
    # intersec_pcd_cam = np.expand_dims(intersec_pcd_cam, axis=0)
    # coord = cam.project_pts(intersec_pcd_cam)
    # # coord = proj(intersec_pcd, rot, t, intr)
    # print('pcd:', coord)
    # cv2.circle(img, (coord[0][0], coord[0][1]), pt_size+2, (255, 255, 255), thickness=-1)

    # # project plane intersection of camera frame
    # intersec_cam = pd.read_csv('../data/output/plane_normals_and_intersection_cam.csv', index_col=0).to_numpy()[-1]
    # intersec_cam = np.expand_dims(intersec_cam, axis=0)
    # coord = cam.project_pts(intersec_cam)
    # print('cam:', coord)
    # cv2.circle(img, (coord[0][0], coord[0][1]), pt_size, (0, 0, 0), thickness=-1)

    # save reprojected image
    cv2.imwrite(args.output, img)
