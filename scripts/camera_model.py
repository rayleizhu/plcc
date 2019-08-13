# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function

from abc import ABCMeta, abstractmethod
import numpy as np

class Camera(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def space_to_plane(self, p_3d):
        return

class PinholeCam(Camera):
    def __init__(self, proj_param, dist_param):
        """
        args:
            proj_param: dictionary 
            dist_param: dictionary 
        """
        self.cx = proj_param['cx']
        self.cy = proj_param['cy']
        self.fx = proj_param['fx']
        self.fy = proj_param['fy']
        self.k1 = dist_param['k1']
        self.k2 = dist_param['k2']
        self.p1 = dist_param['p1']
        self.p2 = dist_param['p2']

    # https://github.com/hengli/camodocal/blob/master/src/camera_models/PinholeCamera.cc
    def _distortion(self, p_u):
        x2 = p_u[:, 0]**2
        y2 = p_u[:, 1]**2
        xy = p_u[:, 0] * p_u[:, 1]
        r2 = x2 + y2
        rad_dist_u = self.k1 * r2 + self.k2 * (r2**2)
        dx = rad_dist_u * p_u[:, 0] + 2 * self.p1 * xy + \
             self.p2 * (r2 + 2 * x2)
        dy = rad_dist_u * p_u[:, 1] + 2 * self.p2 * xy + \
             self.p1 * (r2 + 2 * y2)
        return np.stack([dx, dy], axis=1)

    def space_to_plane(self, p_3d):
        p_3d_front = p_3d[p_3d[:, 2] > 1e-3]  # z > 0
        p_u = p_3d[:, 0:2] / np.expand_dims(p_3d_front[:, 2], 1)
        p_d = p_u + self._distortion(p_u)
        coord_x = self.fx * p_d[:, 0] + self.cx
        coord_y = self.fy * p_d[:, 1] + self.cy
        return np.stack([coord_x, coord_y], axis=1).astype(int)

class MEICam(Camera):
    def __init__(self, proj_param, dist_param, mirr_param):
        """
        args:
            proj_param: dictionary 
            dist_param: dictionary
            mirr_param: float
        """
        self.xi = mirr_param
        # distortion parameters
        self.k1 = dist_param['k1']
        self.k2 = dist_param['k2']
        self.p1 = dist_param['p1']
        self.p2 = dist_param['p2']
        # projection parameters
        self.gamma1 = proj_param['gamma1']
        self.gamma2 = proj_param['gamma2']
        self.u0 = proj_param['u0']
        self.v0 = proj_param['v0']
    
    def _distortion(self, p_u):
        x2 = p_u[:, 0]**2
        y2 = p_u[:, 1]**2
        xy = p_u[:, 0] * p_u[:, 1]
        r2 = x2 + y2
        rad_dist_u = self.k1 * r2 + self.k2 * (r2**2)
        dx = rad_dist_u * p_u[:, 0] + 2 * self.p1 * xy + \
             self.p2 * (r2 + 2 * x2)
        dy = rad_dist_u * p_u[:, 1] + 2 * self.p2 * xy + \
             self.p1 * (r2 + 2 * y2)
        return np.stack([dx, dy], axis=1)


    def space_to_plane(self, p_3d):
        p_3d_front = p_3d[p_3d[:, 2] > 1e-3]  # z > 0
        z = p_3d[:, 2] + self.xi * np.linalg.norm(p_3d_front, axis=1)
        p_u = p_3d[:, 0:2] / np.expand_dims(z, 1)
        p_d = p_u + self._distortion(p_u)
        coord_x = self.gamma1 * p_d[:, 0] + self.u0
        coord_y = self.gamma2 * p_d[:, 1] + self.v0
        return np.stack([coord_x, coord_y], axis=1).astype(int)

