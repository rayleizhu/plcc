# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function

import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np
import os

def get_tag_plane_in_cam_frame(r, t, direction='tag2cam'):
    """
    args:
        r: np.ndarray, 3*3 rotation matrix
        t: np.ndarray, 3d translation vector
        direction: transformation direction, 'cam2tag' means
        p_tag = r \cdot p_cam + t, 'tag2cam' means reverse
    return:
        coeff: coefficient array [A, B, C, D]
    the palne equation can be represented as Ax + By + Cz + D = 0
    """
    assert direction in ['cam2tag', 'tag2cam']
    if direction == 'tag2cam':
        t = -np.dot(r.T, t)
        r = r.T
    e3 = np.array([0, 0, 1])
    n = np.dot(r.T, e3)
    d = np.dot(e3, t)
    return np.append(n, d)

def get_tfm_from_csv(csv_path):
    df = pd.read_csv(csv_path, index_col=0).to_numpy()
    r_list = []
    t_list = []
    for i in range(len(df)):
        t = df[i, 0:3]
        quat = df[i, 3:]
        r = R.from_quat(quat).as_dcm()
        t_list.append(t)
        r_list.append(r)
    return r_list, t_list

def get_tfm_from_rosbag(bag_path):
    pass


if __name__ == '__main__':
    import argparse
    import utility
    from mayavi import mlab

    parser = argparse.ArgumentParser(description="Find planes in camera frame.")
    parser.add_argument('-i', '--input', type=str, default='../data/output/tf_tag_to_cam.csv',
                        help="Input file including transformations from tag frame to camera frame.")
    parser.add_argument('-o', '--output', type=str, default='../data/output/planes_cam.csv',
                        help="Where to put the result.")
    parser.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                         help='Silent mode')
    parser.set_defaults(verbose=True)
    parser.add_argument('-v', '--vis', dest='vis', action='store_true',
                         help='Visualize result')
    parser.set_defaults(vis=False)
    parser.add_argument('-c', '--cos-th', type=float, default=0.85,
                        help='acceptance threshold for intersection.')

    args = parser.parse_args()

    r_list, t_list = get_tfm_from_csv(args.input)
    num_planes = len(r_list)
    coeff_list = []
    
    for i in range(num_planes):
        coeff = get_tag_plane_in_cam_frame(r_list[i], t_list[i], direction='tag2cam')
        coeff_list.append(coeff)

    utility.save_planes(coeff_list, 
                        ['tag{:d}'.format(i) for i in range(num_planes)],
                        args.output)

    res_list = utility.get_all_intersections(coeff_list, cos_th=args.cos_th)
    valid_intersec_list = []
    for res in res_list:
        if res[0]:
            valid_intersec_list.append(res[1])
    if not valid_intersec_list:
        print('====================================================\n'
              '+        WARNING: No valid intersection point:     +\n'
              '+ please make sure intersection angles between tag +\n'
              '+       planes are not too large or too small.     +\n'
              '====================================================')

    if args.verbose:
        print('Transformation from tags to camera\n'
            '(in this way, t is position of tag in camera frame):')
        for i in range(num_planes):
            rot_deg = R.from_dcm(r_list[i].T).as_euler('zyx', degrees=True)
            translation = -np.dot(r_list[i].T, t_list[i])
            print('tag{:02d}: r={}, t={}'.format(i, rot_deg, translation))
        print('---------------------------------------------------')
        print('Coefficient array [n.x, n.y, n.z, d] of tag planes:')
        for i in range(num_planes):
            print('tag{:02d}:'.format(i), coeff_list[i])
        print('Result is stored to %s' % args.output)
        print('---------------------------------------------------')
        print('Intersections:')
        [print(res) for res in res_list]

    if args.vis and valid_intersec_list:
        valid_intersec = np.stack(valid_intersec_list)
        slack = 0.5    
        bound_min = np.min(valid_intersec, axis=0) - slack
        bound_max = np.max(valid_intersec, axis=0) + slack
        surf_list = utility.visualize_planes(coeff_list, bound_min, bound_max)
        mlab.points3d(valid_intersec[:, 0], valid_intersec[:, 1], valid_intersec[:, 2],
                    scale_factor=0.03,
                    opacity=1.0,
                    color=(0., 0., 0.))
        mlab.show()
    