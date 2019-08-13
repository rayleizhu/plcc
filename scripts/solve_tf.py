# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function

import pandas as pd
import scipy.optimize
from scipy.spatial.transform import Rotation as R
import numpy as np
import argparse
import os

import utility

def gen_cost_func(P, Q):
    """
    Q = R(theta_z, theta_y, theta_x) \cdot P
    return:
        cost_fn: a callable object that return cost for given input
                 euler_angle (theta_z, theta_y, theta_x)
    """
    def cost_fn(angle):
        r = R.from_euler('zyx', angle, degrees=True)
        tf_mat = r.as_dcm()
        Q_tf = np.dot(tf_mat, P)
        return np.linalg.norm(Q_tf - Q)
    return cost_fn

def get_tfm_via_plane_constraint(source_planes, target_planes, init_euler=[0, 0, 0], cos_th=0.85):
    """
    args:
        source_planes: (n, 4) coefficient array of n planes in source frame
        target_planes: (n, 4) coefficient array of n planes in target frame
        init_euler: initial value for optimization
        cos_th: threshold for intersection acceptance
    return:
        (r, t, res): rotation matrix, translation and complete optimization result report
    """
    assert len(source_planes) == len(target_planes)
    num_planes = len(source_planes)

    source_normals = source_planes[:, 0:3]
    target_normals = target_planes[:, 0:3]
    cost_fn = gen_cost_func(source_normals.T, target_normals.T)

    res = scipy.optimize.minimize(cost_fn, init_euler)
    r = R.from_euler('zyx', res.x, degrees=True).as_dcm()

    source_intersec_list = utility.get_all_intersections(source_planes, cos_th)
    target_intersec_list = utility.get_all_intersections(target_planes, cos_th)

    source_valid_list = []
    target_valid_list = []
    for i in range(len(source_intersec_list)):
        if source_intersec_list[i][0] and target_intersec_list[i][0]:
            source_valid_list.append(source_intersec_list[i][1])
            target_valid_list.append(target_intersec_list[i][1])
    if not source_valid_list:
        print('====================================================\n'
              '+        WARNING: No valid intersection point:     +\n'
              '+ please make sure intersection angles between tag +\n'
              '+       planes are not too large or too small.     +\n'
              '====================================================')
        raise ValueError('No valid intersection points!')
 
    source_valid = np.stack(source_valid_list)
    target_valid = np.stack(target_valid_list)
    t = np.mean(target_valid.T - np.dot(r, source_valid.T), axis=1)

    return r, t, res, source_intersec_list, target_intersec_list

def main():

    parser = argparse.ArgumentParser(description="Solve transfromation via normal and intersection constraints.")
    parser.add_argument('-s', '--source', type=str, default='../data/output/planes_pcd.csv',
                        help="csv file containing planes in source frame.")
    parser.add_argument('-t', '--target', type=str, default='../data/output/planes_cam.csv',
                        help="csv file containing planes in target frame.")
    parser.add_argument('-o', '--output', type=str, default='../data/output/calibration_result.csv',
                        help="output file for calibration result.")
    parser.add_argument('--itsc-src', type=str, default='../data/output/intersection_src.csv',
                        help="output file for intersections in camera frame.")
    parser.add_argument('--itsc-tgt', type=str, default='../data/output/intersection_tgt.csv',
                        help="output file for intersections in lidar frame.")
    parser.add_argument('-q', '--quiet', dest='verbose', action='store_false', help='Work sliently')
    parser.set_defaults(verbose=True)
    parser.add_argument('-c', '--cos-th', type=float, default=0.85,
                        help='acceptance threshold for intersection.')

    args = parser.parse_args()
    source_planes = pd.read_csv(args.source, index_col=0).to_numpy()
    target_planes = pd.read_csv(args.target, index_col=0).to_numpy()

    source_normals = source_planes[:, 0:3]
    target_normals = target_planes[:, 0:3]

    init_dcm = np.dot(np.dot(target_normals, source_normals.T),
                      np.linalg.inv(np.dot(source_normals.T, source_normals)))
    init_r = R.from_dcm(init_dcm)
    init_euler = init_r.as_euler('zyx', degrees=True)

    #init_euler = [0, 0, 0]
    r, t, report, src_itsc, tgt_itsc = get_tfm_via_plane_constraint(source_planes,
                                                                    target_planes,
                                                                    init_euler=init_euler,
                                                                    cos_th=args.cos_th)
    
    rot_quat = R.from_euler('zyx', report.x, degrees=True).as_quat()
    output = np.append(t, rot_quat)
    df_calib_res = pd.DataFrame(data=np.expand_dims(output, axis=0), columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df_calib_res.to_csv(args.output, index=False)

    itsc_src_valid = []
    for itsc in src_itsc:
        if itsc[0]:
            itsc_src_valid.append(itsc[1])
    df_itsc_src_valid = pd.DataFrame(data=np.stack(itsc_src_valid),
                                     columns=['x', 'y', 'z'])
    df_itsc_src_valid.to_csv(args.itsc_src, index=False)

    itsc_tgt_valid = []
    for itsc in tgt_itsc:
        if itsc[0]:
            itsc_tgt_valid.append(itsc[1])
    df_itsc_tgt_valid = pd.DataFrame(data=np.stack(itsc_tgt_valid),
                                     columns=['x', 'y', 'z'])
    df_itsc_tgt_valid.to_csv(args.itsc_tgt, index=False)

    if args.verbose:
        print('---------------------------------------------------')
        print('Round robin dot product between normals:')
        print('++in source frame:')
        print(np.dot(source_normals, source_normals.T))
        print('++in target frame:')
        print(np.dot(target_normals, target_normals.T))
        print('---------------------------------------------------')
        print('Intersection points:')
        print('++in source frame:')
        [print(pt) for pt in src_itsc]
        print('++in target frame:')
        [print(pt) for pt in tgt_itsc]
        print('Intersection points saved to {} and {}'.format(args.itsc_src, args.itsc_tgt))
        print('---------------------------------------------------')
        print('Optimization report:')
        print('++initialization: ', init_euler)
        print('++result: ')
        print(report)
        print('---------------------------------------------------')
        print('Inner product between transformed normals and target normals:')
        tfed_normals = np.dot(r, source_normals.T).T
        print(np.dot(tfed_normals, target_normals.T))
        print('---------------------------------------------------')
        print('Final output:')
        print('++rotation in degree zyx: ', report.x)
        print('++rotation (qx, qy, qz, qw): ', rot_quat)
        print('++translation (x, y, z): ', t)
        print('Result is saved to %s'%(args.output))

if __name__ == '__main__':
    main()