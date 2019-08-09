# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import numpy as np
from sklearn.decomposition import PCA
import pandas as pd
import copy
import scipy
import os


class Plane3D:
    def __init__(self, normal=np.zeros(3), center=np.zeros(3)):
        self.normal = normal
        self.center = center
        self.fitted = False
        if (normal != np.zeros(3)).all():
            self.fitted = True
        
    def fit(self, pcd, verbose=False):
        pca = PCA(n_components=3)
        pca.fit(pcd)
        self.center = np.mean(pcd, axis=0)
        self.normal = pca.components_[-1]
        if verbose:
            print("variance ratio:", pca.explained_variance_ratio_)
            print("variance:", pca.explained_variance_)
            print("three axes:")
            print(pca.components_)
            print("pcd center:", self.center)
            print("normal:", pca.components_[-1])
        self.fitted = True
        return self

    def get_parameter(self, mode='p-n'):
        assert mode in ['p-n', 'coeff']
        assert self.fitted
        if mode == 'p-n':
            return self.normal, self.center
        else:
            coeff = np.append(self.normal,
                              -np.dot(self.normal, self.center))
            return coeff

    def get_error(self, points):
        assert self.fitted
        error = np.dot(points - self.center, self.normal)
        return error**2


class Ransac:
    """
    reference: http://scipy.github.io/old-wiki/pages/Cookbook/RANSAC
    """
    def __init__(self, min_samples, max_iter, inlier_th, num_inliers_th):
        self.min_samples = min_samples
        self.max_iter = max_iter
        self.inlier_th = inlier_th
        self.num_inliers_th = num_inliers_th

    def run(self, model, data, verbose=False, return_all=True):
        iterations = 0
        best_model = None
        best_err = np.inf
        best_inliner_idxs = None

        if self.num_inliers_th > 0. and self.num_inliers_th < 1.:
            d = self.num_inliers_th * len(data)
        elif self.num_inliers_th > 1:
            d =  self.num_inliers_th
        else:
            raise ValueError('Illegal value for num_inliers_th!')

        if self.min_samples > 0. and self.min_samples < 1.:
            n = int(self.min_samples * len(data))
        elif self.min_samples > 1:
            n =  self.min_samples
        else:
            raise ValueError('Illegal value for min_samples!')

        while iterations < self.max_iter:
            all_idxs = np.arange(len(data))
            np.random.shuffle(all_idxs)
            maybe_idxs = all_idxs[:n]
            other_idxs = all_idxs[n:]
            maybe_inliners = data[maybe_idxs, :]
            other_points = data[other_idxs, :]
            model.fit(maybe_inliners)
            test_err = model.get_error(other_points)
            also_idxs = other_idxs[test_err < self.inlier_th]
            also_inliers = data[also_idxs, :]

            inliers = np.concatenate([maybe_inliners, also_inliers])

            if len(inliers) >= d:
                model.fit(inliers)
                cur_model = copy.deepcopy(model)
                cur_err = np.mean(model.get_error(inliers))
                if cur_err < best_err:
                    best_model =  cur_model
                    best_err = cur_err
                    best_inliner_idxs = np.concatenate([maybe_idxs, also_idxs])

            if verbose:
                print('iteration [%d] -- min test error: %f -- max test error: %f -- num_inliers: %d/%d = %f' %
                      (iterations, test_err.min(), test_err.max(), len(inliers), len(data), len(inliers)/len(data)))
            iterations += 1

        if best_model is None:
            raise ValueError("Did not meet acceptance criteria.")
        if return_all:
            return best_model, data[best_inliner_idxs, :]
        else:
            return best_model
                

def statistical_outilier_removal(data, k=8, z_max=2):
    """
    reference: https://stackoverflow.com/questions/38754668/plane-fitting-in-a-3d-point-cloud
    """
    kdtree = scipy.spatial.cKDTree(data, copy_data=True)
    distances, i = kdtree.query(kdtree.data, k=k, n_jobs=-1)
    z_distances = scipy.stats.zscore(np.mean(distances, axis=1))
    sor_filter = abs(z_distances) < z_max
    return data[sor_filter]


def get_tag_plane_in_pcd_frame(pts, sor=True, ransac=False, verbose=False):
    """
    args:
        pts: ndarray of shape (n, 3), points in the plane
        sor: flag for using statistical outlier filter, recommended to turn on
        ransac: flag for using RANSAC, NOT recommened to turn on, since it's not stable
        verbose: flag for verbose output
    return:
        coeff: coefficent array [n.x, n.y, n.z, d]
    """
    if args.sor == True:
        # TODO: user control on filter condition
        pts = statistical_outilier_removal(pts, k=10, z_max=2)
    if args.ransac == True:
        plane, inliers = Ransac(0.5, 100, 1e-3, 0.9).run(Plane3D(), pts, verbose=verbose)
    else:
        inliers = pts
        plane = Plane3D().fit(pts, verbose=verbose)
    coeff = plane.get_parameter(mode='coeff')
    return coeff, inliers

if __name__ == '__main__':

    from mayavi import mlab
    import argparse
    import utility

    parser = argparse.ArgumentParser(description="Fit planes with point cloud patches.")
    parser.add_argument('-i', '--input', type=str, default='../data/output/pcd_patches',
                        help="The directory including point cloud patches.")
    parser.add_argument('-o', '--output', type=str, default='../data/output/planes_pcd.csv',
                        help="Where to put the result.")
    parser.add_argument('--no-sor', dest='sor', action='store_false', help='Disable statistical outlier removal.')
    parser.add_argument('--ransac', dest='ransac', action='store_true', help='Enable RANSAC for plane fitting.')
    parser.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                         help='Silent mode')
    parser.add_argument('--no-vis', dest='vis', action='store_false', help='Disable visualization')
    parser.add_argument('-c', '--cos-th', type=float, default=0.85,
                        help='acceptance threshold for intersection.')

    parser.set_defaults(sor=True)
    parser.set_defaults(ransac=False)
    parser.set_defaults(verbose=True)
    parser.set_defaults(vis=True)

    args = parser.parse_args()

    #csv_file_list = [ 'selected_pcd_patch_%d.csv'%i for i in range(1, 4) ]
    csv_name_list = os.listdir(args.input)
    csv_name_list.sort()
    if args.verbose:
        print('Point cloud patch csv files:\n', csv_name_list)
        print('---------------------------------------------------')
    csv_file_list = [ os.path.join(args.input, csv_name) for csv_name in csv_name_list ]
    num_planes = len(csv_file_list)

    plane_list = []
    inliers_list = []
    for i, csv in enumerate(csv_file_list):
        if args.verbose:
            print('--------Fitting plane %d---------'%i)
        pts = pd.read_csv(csv).to_numpy()
        coeff, inliers = get_tag_plane_in_pcd_frame(pts,
                                                    sor=args.sor,
                                                    ransac=args.ransac,
                                                    verbose=args.verbose)
        # It's assumed that, x axis points towards the positive side of plane
        if coeff[0] > 0:
            coeff = -coeff
        plane_list.append(coeff)
        inliers_list.append(inliers)
        if args.verbose:
            print('---------------------------------------------------')

    index = ['tag{:02d}'.format(i) for i in range(len(plane_list))]
    utility.save_planes(plane_list, index, args.output)

    res_list = utility.get_all_intersections(plane_list, cos_th=args.cos_th)
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
        print('---------------------------------------------------')
        print('Coefficient array [n.x, n.y, n.z, d] of tag planes:')
        for i in range(num_planes):
            print('tag{:02d}:'.format(i), plane_list[i])
        print('Result is stored to %s' % args.output)
        print('---------------------------------------------------')
        print('Intersections:')
        [print(res) for res in res_list]

    if args.vis:
        red_ext = np.linspace(0, 1, num=num_planes)
        color_map = [(red_ext[i], 0, 1-red_ext[i]) for i in range(num_planes)]
        slack = 0.01
        surf_list = []
        for i in range(num_planes):
            bound_min = np.min(inliers_list[i], axis=0) - slack
            bound_max = np.max(inliers_list[i], axis=0) + slack
            surf_list += utility.visualize_planes([plane_list[i]], bound_min, bound_max, [color_map[i]])
            # TODO: Visualize both inliers and outliers?
            mlab.points3d(inliers_list[i][:, 0],
                          inliers_list[i][:, 1],
                          inliers_list[i][:, 2], 
                          color=color_map[i])
        
        
        if valid_intersec_list:
            valid_intersec = np.stack(valid_intersec_list)
            mlab.points3d(valid_intersec[:, 0], valid_intersec[:, 1], valid_intersec[:, 2],
                        scale_factor=0.03,
                        opacity=1.0,
                        color=(0., 0., 0.))

        mlab.show()
