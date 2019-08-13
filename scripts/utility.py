# Author: Zhu Lei
# Email: leifzhu@foxmail.com

from __future__ import print_function

from mayavi import mlab
import os
import pandas as pd
import numpy as np

def get_color_map(n, mode='rgb'):
    assert mode in ['rgb', 'bgr']
    intens = np.linspace(0, 1, num=n)
    if mode == 'rgb':
        color_map = [(intens[i], 0, 1-intens[i]) for i in range(n)]
    elif mode == 'bgr':
        color_map = [(1-intens[i], 0, intens[i]) for i in range(n)]
    else:
        raise ValueError("only support 'rgb' or 'bgr' as mode!")
    return color_map

def visualize_planes(coeff_list, bound_min, bound_max, color_map=None):
    """
    args:
        coeff_list: list of coefficient array
        bound_min: tuple or ndarray, (x_min, y_min, z_min)
        bound_max: tuple or ndarray, (x_max, y_max, z_max)
    return:
        surf_list: a list of mlab surface objects
    """
    # FIXME: plane parallel to z axis
    #https://stackoverflow.com/questions/53115276/matplotlib-how-to-draw-a-vertical-plane-in-3d-figure
    if color_map is None:
        color_map = get_color_map(len(coeff_list))
    surf_list = []
    for i, coeff in enumerate(coeff_list):
        def zz(x, y):
            return (-coeff[3] - coeff[0]*x - coeff[1]*y) / coeff[2]
        xx, yy = np.mgrid[bound_min[0]:bound_max[0]:5j,
                          bound_min[1]:bound_max[1]:5j]
        surf = mlab.surf(xx, yy, zz, color=color_map[i], opacity=0.5)
        surf_list.append(surf)
    return surf_list

def save_planes(coeff_list, index, path):
    coeff = np.stack(coeff_list)
    df = pd.DataFrame(coeff, columns=['nx', 'ny', 'nz', 'd'], index=index)
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    df.to_csv(path)


def get_3plane_intersection(coeff_list, cos_th=0.85):
    """
    args:
        coeff_list: list containing 3 coefficient arrays
        cos_th: threshold for stable solution, set to value > 1 means
                accept any case
    return:
        q: intersection of 3 planes
    """
    assert len(coeff_list) == 3
    stable = True
    n_list = [coeff[0:3]/np.linalg.norm(coeff[0:3]) for coeff in coeff_list]
    c_list = [-coeff[3] for coeff in coeff_list]
    for i in range(3):
        abs_cos = np.abs(np.dot(n_list[i%3], n_list[(i+1)%3]))
        stable =  stable and (abs_cos < cos_th)
    if stable:
        N = np.stack(n_list)
        c = np.stack(c_list)
        q = np.dot(np.linalg.inv(N), c)
    else:
        q = np.zeros(3)
    return stable, q

def get_all_intersections(coeff_list, cos_th=0.85):
    """
    args:
        coeff_list: (n, 4) array or list of coefficient array
    return:
        a list of tuples, each tuple is like (stable_flag, intersection_pt)
    """
    num_planes = len(coeff_list)
    res_list = []
    for i in range(num_planes-2):
        for j in range(i+1, num_planes-1):
            for k in range(j+1, num_planes):
                coeff_3plane = [coeff_list[x] for x in (i, j, k)]
                res = get_3plane_intersection(coeff_3plane,
                                                       cos_th=cos_th)
                res_list.append(res)
    return res_list

