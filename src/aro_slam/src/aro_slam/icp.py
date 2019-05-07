from __future__ import absolute_import, division, print_function

import numpy as np
from math import sqrt
from scipy.spatial import cKDTree
import cv2
import math
def func(x, a, b):
    return a + b*b*x  # Term b*b will create bimodality.


def icp(x, y, num_iters=10, inlier_ratio=1.0, inlier_dist_mult=1.0, tol=1e-3, y_index=None, R=None, t=None):
    """Iterative closest point algorithm, minimizing sum of squares of point to point distances.

    :param x: Points to align, D-by-N matrix.
    :param y: Reference points to align to, such that y[:, j] ~ R * x[:, i] + t for corresponding pair (i, j).
    :param num_iters: Number of iterations, consisting of NN search and transform update.
    :param inlier_ratio: Ratio of nearest points from which to compute inlier distance.
    :param inlier_dist_mult: Inlier distance multiplier deciding what is included in optimization.
    :param tol: Minimum improvement per iteration to continue.
    :param y_index: Index for NN search for y (cKDTree).
    :param R: Initial rotation.
    :param t: Initial translation.
    :return: Optimized rotation R, translation t, and corresponding criterion value on inliers (including multiplier).
   """
    #y = []
    #x = []


    mojeR = np.array([[math.cos(np.deg2rad(0)),math.sin(np.deg2rad(0))],[math.sin(np.deg2rad(0)),math.cos(np.deg2rad(0))]])
    mojet = np.array([[1],[4]])

    #y = np.array([[1,2,3],[1,1,1]])
    #x = np.array([[2,3,4],[2,3,4]])
    #x = np.array([[1,2,3],[0,0,0]])
    #y = np.matmul(mojeR,x) + mojet
    #R = np.eye(x.shape[0])
    #t = np.zeros((x.shape[0], 1))
    y_index = cKDTree(y.T)

    bestErr = 10000000
    inlier_ratio = 0.8
    if y_index is None:
        y_index = cKDTree(y.T)
    if R is None:
        R = np.eye(x.shape[0])
    if t is None:
        t = np.zeros((x.shape[0], 1))
    prev_err_inl = float('inf')
    RR = R
    tt = t
    for i in range(num_iters):
        filt = 0
        dist = 0
        x = (np.matmul(R,x)+t)
        [dist, index] = y_index.query(x.T, )
        filt = inlier_dist_mult*np.quantile(dist,inlier_ratio)
        X = x[:, dist<=filt]
        Y = y[:, index[dist<=filt]]
        centroid_x = np.mean(X, axis=1, keepdims = True)
        centroid_y = np.mean(Y, axis=1, keepdims = True)
        YY = Y - centroid_y
        XX = X - centroid_x
        H = np.matmul(XX, YY.T)
        [U, Sigma, V] = np.linalg.svd(H,full_matrices = True)
        R = np.matmul(V.T,U.T)
        t = centroid_y - np.matmul(R,centroid_x)
        prev_err_inl = 0
        #[dist, index] = y_index.query((np.matmul(R,x)+t).T)
        Xtemp = (np.matmul(R,X)+t)
        errtmp = 0
        errtmp= (Xtemp.T - Y.T)**2
        for err in errtmp:
            prev_err_inl += err
        prev_err_inl = np.linalg.norm(prev_err_inl)
        if prev_err_inl < bestErr:
            bestErr = prev_err_inl
        else:
            break
        RR = np.matmul(R,RR)
        tt = np.matmul(R,tt) + t
    print("best:")
    print(tt)
    print(RR)
    return RR, tt, bestErr
