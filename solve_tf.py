# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import pandas as pd
import scipy.optimize
from scipy.spatial.transform import Rotation as R
import numpy as np
from mayavi import mlab

def gen_cost_func(P, Q):
    def cost(angle):
        r = R.from_euler('zyx', angle, degrees=True)
        tf_mat = r.as_dcm()
        Q_tf = np.dot(tf_mat, P)
        return np.linalg.norm(Q_tf - Q)
    return cost

def main():
    data_cam = pd.read_csv('plane_normals_and_intersection_cam.csv', index_col=0)
    data_pcd = pd.read_csv('plane_normals_and_intersection_pcd.csv', index_col=0)
    source = data_pcd.to_numpy()
    target = data_cam.to_numpy()
    print('round robin dot product between normals:')
    normal_source = source[0:3].T
    normal_target = target[0:3].T
    print("source:")
    print(np.dot(normal_source.T, normal_source))
    print('target:')
    print(np.dot(normal_target.T, normal_target))
    print('==================================')
    cost_fun = gen_cost_func(normal_source, normal_target)
    # init_dcm = np.dot(np.linalg.inv(np.dot(normal_source, normal_source.T)),
    #               np.dot(normal_target, normal_source.T))
    # init_r = R.from_dcm(init_dcm)
    # init_euler = init_r.as_euler('zyx', degrees=True)
    init_euler = [0, 0, 0]
    print('initialization: ', init_euler)
    #bounds = [(-180, 180) for i in range(3)]
    res = scipy.optimize.minimize(cost_fun, init_euler)
    print('optimization result:')
    print(res)
    print('inner product between transformed normals and target normals:')
    r = R.from_euler('zyx', res.x, degrees=True)
    rot_mat = r.as_dcm()
    normal_tfed = np.dot(rot_mat, normal_source)
    print(np.dot(normal_tfed.T, normal_target))
    print('==================================')
    rot_quat = r.as_quat()
    t = target[-1] - np.dot(rot_mat, source[-1])
    print('rotation (qx, qy, qz, qw):', rot_quat)
    print('translation (x, y, z):', t)
    output = np.append(t, rot_quat)
    df = pd.DataFrame(data=np.expand_dims(output, axis=0), columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df.to_csv('calib_result_pcd_to_cam.csv')

if __name__ == '__main__':
    main()