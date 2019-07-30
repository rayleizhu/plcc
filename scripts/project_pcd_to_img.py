# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import pandas as pd
import mei_model

class Camera:
    def __init__(self, intr, cam_model='pinhole', **kwargs):
        """
        args:
            intr: [3, 3] intrisic matrix
        """
        assert cam_model in ['pinhole', 'mei']
        self.intr = intr
        self.cam_model = cam_model

    def project_pts(self, pts):
        """
        args:
            pts: (n, 3) numpy array
        return:
            coord: (n, 2) np array
        """
        if self.cam_model == 'pinhole':
            pts_cam_front = pts[pts[:, 2] > 1e-3]  # z > 0
            print(pts_cam_front)
            coord_hom = np.dot(intr, pts_cam_front.T).T
            coord = coord_hom[:, 0:2] / np.expand_dims(coord_hom[:, 2], 1)
            # print(coord)
            coord = coord.astype(int)
            # print(coord)
        elif self.cam_model == 'mei':
            pts_cam_front = pts[pts[:, 2] > 1e-3]  # z > 0
            m_params = intr
            coord = mei_model.space_to_plane_mei(pts_cam_front, m_params)  # image point coordinates
            print(coord)
            coord = coord.astype(int)
        else:
            raise NotImplementedError("Camera model %s is not implemented!" % (self.cam_model))
        return coord


# intr = np.array([1361.3861083984375, 0.0, 949.724853515625, 0.0, 1361.753173828125, 545.7684936523438, 0.0, 0.0, 1.0]).reshape(3, 3)
m_params = mei_model.read_yaml_file()
print(m_params)
m_inv_params = mei_model.mei_model_camera_constructor(m_params)  # 构造mei模型
print(m_inv_params)
intr = m_params
# intr = np.array([m_params['m_gamma1'], 0.0, m_params['m_u0'], 0.0, m_params['m_gamma2'], m_params['m_v0'], 0.0, 0.0, 1.0]).reshape(3, 3)


extr_csv = '../data/output/calib_result.csv'
extr = pd.read_csv(extr_csv).to_numpy()[0]
rot = R.from_quat(extr[3:]).as_dcm()
t = extr[0:3]

img = cv2.imread('../data/output/orig.png')

csv_file_list = ['../data/output/pcd_patches/selected_pcd_patch_%d.csv' % i for i in range(1, 4)]
pt_size = 3
color_ls = [(255 * (i == 2), 255 * (i == 1), 255 * (i == 0)) for i in range(3)]


def proj(pts_pcd, rot, t, intr):
    pts_cam = (np.dot(rot, pts_pcd.T) + np.expand_dims(t, 1)).T
    pts_cam_front = pts_cam[pts_cam[:, 2] > 1e-3]  # z > 0
    coord_hom = np.dot(intr, pts_cam_front.T).T
    coord = coord_hom[:, 0:2] / np.expand_dims(coord_hom[:, 2], 1)
    coord = coord.astype(int)
    return coord


cam = Camera(intr, cam_model='mei')

# project point cloud
for i, fname in enumerate(csv_file_list):
    pts_pcd = pd.read_csv(fname).to_numpy()
    pts_cam = (np.dot(rot, pts_pcd.T) + np.expand_dims(t, 1)).T
    coord = cam.project_pts(pts_cam)
    for j in range(len(coord)):
        cv2.circle(img, (coord[j][0], coord[j][1]), pt_size, color_ls[i], thickness=-1)

# print(coord)

# project plane intersection of lidar frame
intersec_pcd = pd.read_csv('../data/output/plane_normals_and_intersection_pcd.csv', index_col=0).to_numpy()[-1]
intersec_pcd_cam = np.dot(rot, intersec_pcd) + t
intersec_pcd_cam = np.expand_dims(intersec_pcd_cam, axis=0)
coord = cam.project_pts(intersec_pcd_cam)
# coord = proj(intersec_pcd, rot, t, intr)
print('pcd:', coord)
cv2.circle(img, (coord[0][0], coord[0][1]), pt_size+2, (255, 255, 255), thickness=-1)

# project plane intersection of camera frame
intersec_cam = pd.read_csv('../data/output/plane_normals_and_intersection_cam.csv', index_col=0).to_numpy()[-1]
intersec_cam = np.expand_dims(intersec_cam, axis=0)
coord = cam.project_pts(intersec_cam)
print('cam:', coord)
cv2.circle(img, (coord[0][0], coord[0][1]), pt_size, (0, 0, 0), thickness=-1)

# save reprojected image
cv2.imwrite('../data/output/back_proj_res.png', img)
