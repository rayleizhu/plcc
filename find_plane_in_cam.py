# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np
from mayavi import mlab

def main():
    df = pd.read_csv('tf_cam_to_tag.csv').to_numpy()
    n_list = []
    d_list = []
    for i in range(3):
        t = df[i, 0:3]
        quat = np.append(df[i, 4:7], df[i, 3])
        #print(quat)
        r = R.from_quat(quat)
        rot_mat = r.as_dcm()
        e3 = np.array([0, 0, 1])
        n = np.dot(rot_mat.T, e3)
        d = -np.dot(e3, t)
        n_list.append(n)
        d_list.append(d)
        # debug
        print('cam posistion in tag %d' % (i+1))
        print(-np.dot(rot_mat.T, t))

    N = np.stack(n_list)
    c = np.stack(d_list)
    intersection =  np.dot(np.linalg.inv(N), c)

    for i in range(3):
        def f(x, y):
            return (c[i] - N[i][0]*x - N[i][1]*y) / N[i][2]
        s = 0.5
        x_min = intersection[0] - s
        y_min = intersection[1] - s
        x_max = intersection[0] + s
        y_max = intersection[1] + s
        step = 2*s / 5
        xx, yy = np.mgrid[x_min:x_max:step, y_min:y_max:step]
        color=tuple([1. if j==i else 0. for j in range(3)])
        mlab.surf(xx, yy, f, color=color, opacity=0.5)
    mlab.points3d([intersection[0]], [intersection[1]], [intersection[2]], scale_factor=0.03)
    mlab.show()

    df = pd.DataFrame(np.append(N, intersection[np.newaxis, :], axis=0),
                      columns=['x', 'y', 'z'],
                      index=['normal_1', 'normal_2', 'normal_3', 'intersection'])
    df.to_csv('plane_normals_and_intersection_cam.csv')

    
if __name__ == '__main__':
    main()