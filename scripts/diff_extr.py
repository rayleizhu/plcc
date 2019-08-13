# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import numpy as np


def get_extr_diff(r1, t1, r2, t2):
    """
    (r1, t1): transformation from common frame to frame 1
    (r2, t2): transformation from common frame to frame 2
    return:
     transformation from frame 1 to frame 2
    """
    rot_diff = np.dot(r2, r1.T)
    t_diff = t2 - np.dot(rot_diff, t1)
    return rot_diff, t_diff

if __name__ == '__main__':
    import argparse
    import os
    import pandas as pd
    from scipy.spatial.transform import Rotation as R

    parser = argparse.ArgumentParser(description="Get relative transformation.")
    parser.add_argument('-s', '--source', type=str, default='../data/output/calib_result_1.csv')
    parser.add_argument('-t', '--target', type=str, default='../data/output/calib_result_2.csv')

    args = parser.parse_args()

    source_extr = pd.read_csv(args.source).to_numpy()[0]
    target_extr = pd.read_csv(args.target).to_numpy()[0]
    r1 = R.from_quat(source_extr[3:]).as_dcm()
    t1 = source_extr[:3]
    r2 = R.from_quat(target_extr[3:]).as_dcm()
    t2 = target_extr[:3]

    r12, t12 = get_extr_diff(r1, t1, r2, t2)
    deg = R.from_dcm(r12).as_euler('zyx', degrees=True)
    print('rotation: {}\ntranslation:{}'.format(deg, t12)) 
