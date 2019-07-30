# Author: Zhu Lei
# Email: leifzhu@foxmail.com

import numpy as np
from sklearn.decomposition import PCA
import pandas as pd
from mayavi import mlab
import copy
import scipy

import argparse
import os

class Plane3D:
    def __init__(self, normal=np.zeros(3), center=np.zeros(3)):
        self.normal = normal
        self.center = center
        self.fitted = False
        if (normal != np.zeros(3)).all():
            self.fitted = True
        self.pcd = None

    def fit(self, pcd, verbose=False):
        self.pcd = pcd
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

    def run(self, data, model, verbose=False, return_all=False):
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
            return best_model, best_inliner_idxs
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

def main():
    parser = argparse.ArgumentParser(description="Fit planes with point cloud patches.")
    parser.add_argument('--input_dir', type=str, default='../data/output/pcd_patches',
                        help="The directory including point cloud patches.")
    parser.add_argument('--output_file', type=str, default='../data/output/plane_normals_and_intersection_pcd.csv',
                        help="Where to put the result.")
    parser.add_argument('--no-sor', dest='sor', action='store_false', help='Disable statistical outlier removal.')
    parser.add_argument('--ransac', dest='ransac', action='store_true', help='Enable RANSAC for plane fitting.')
    parser.add_argument('--verbose', dest='verbose', action='store_true', help='Output verbosely')

    parser.set_defaults(sor=True)
    parser.set_defaults(ransac=False)
    parser.set_defaults(verbose=False)

    args = parser.parse_args()

    # TODO: multiple planes
    #csv_file_list = [ 'selected_pcd_patch_%d.csv'%i for i in range(1, 4) ]
    csv_file_list = [ os.path.join(args.input_dir, 'selected_pcd_patch_%d.csv'%i) for i in range(1, 4) ]

    n_list = []
    c_list = [] 
    for i, csv in enumerate(csv_file_list):
        pts = pd.read_csv(csv).to_numpy()

        if args.verbose:
            print('--------Fitting plane %d---------'%i)

        if args.sor == True:
            # TODO: user control on filter condition
            pts = statistical_outilier_removal(pts, k=10, z_max=2)
        if args.ransac == True:
            plane = Ransac(0.5, 100, 1e-3, 0.9).run(pts, Plane3D(), verbose=args.verbose)
        else:
            plane = Plane3D()
            plane.fit(pts, verbose=args.verbose)

        n, center = plane.get_parameter()
        # TODO: Visualize both inliers and outliers?
        pts = plane.pcd

        # FIXME: the normal direction of planes
        if n[0] > 0:
            n = -n
        n_list.append(n)
        c_list.append(np.dot(n, center))

        # TODO: more robust visualization
        def f(x, y):
            return (np.dot(n, center) - n[0]*x - n[1]*y)/n[2]
        
        s = 0.1
        x_min = np.min(pts[:, 0]) - s
        y_min = np.min(pts[:, 1]) - s
        x_max = np.max(pts[:, 0]) + s
        y_max = np.max(pts[:, 1]) + s
        xx, yy = np.mgrid[x_min:x_max:0.01, y_min:y_max:0.01]
        color=tuple([1. if j==i else 0. for j in range(3)])
        mlab.surf(xx, yy, f, color=color, opacity=0.5)
        mlab.points3d(pts[:, 0], pts[:, 1], pts[:, 2], 
                     color=color)

    N = np.stack(n_list)
    c = np.stack(c_list)
    intersection = np.dot(np.linalg.inv(N), c)
    mlab.points3d([intersection[0]], [intersection[1]], [intersection[2]], scale_factor=0.03)
    mlab.show()


    if not os.path.exists(os.path.dirname(args.output_file)):
        os.makedirs(os.path.dirname(args.output_file))

    df = pd.DataFrame(np.concatenate([N, intersection[np.newaxis, :]], axis=0),
                      columns=['x', 'y', 'z'],
                      index=['normal_1', 'normal_2', 'normal_3', 'intersection'])
    df.to_csv(args.output_file)

    if args.verbose:
        print('Result is stored to %s' % args.output_file)

if __name__ == '__main__':
    main()