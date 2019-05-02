from __future__ import absolute_import, division, print_function
import numpy as np
from scipy.spatial import cKDTree
from random import *
import math


def icp(x, y, num_iters=10, inlier_ratio=1, inlier_dist_mult=1.0, tol=1e-3, y_index=None, R=None, t=None):
    """Iterative closest point algorithm, minimizing sum of squares of point to point distances.

    :param x: Points to align, D-by-N matrix.  D = dimension, N = pocet pointu
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
    assert x.shape[0] == y.shape[0]
    if y_index is None:
        y_index = cKDTree(y.T)
    if R is None:
        R = np.eye(x.shape[0])
    if t is None:
        t = np.zeros((x.shape[0], 1))
    prev_err_inl = float('inf')
    # inlier_ratio = 0.8
    # pocet sousedu
    k = 1
    R_tmp = R
    t_tmp = t

    for i in range(num_iters):
        
        x = (np.matmul(R_tmp,x)+t_tmp)
        C, D = findCorespondences(x, y, y_index, k)
        x_tmp, c_tmp, prev_err_inl = findInlierCorrespondences(x, D, C, inlier_ratio, inlier_dist_mult)
        R_tmp, t_tmp = coundRt(x_tmp, c_tmp)

        R =  np.matmul(R_tmp, R)
        t = np.matmul(R_tmp,t) + t_tmp

        if prev_err_inl <= tol:
            break


    print("BEST")
    print("R")
    print(R)
    print("t")
    print(t)

    return R, t, prev_err_inl
    
# ref, P must be ref.T !!!, 
def findCorespondences (P, ref, tree, k):
    ref = ref.T
    P = P.T
    D, idx = tree.query(P)
    C = ref[idx]
    C = C.T
    return C, D

def findInlierCorrespondences(x, d, c, p, r):
    Qd = np.quantile(d, p)
    counter = 0
    for i in range(0, x.shape[1]):
        if (d[i-counter] <= r*Qd):
            x = np.delete(x, i-counter, 1)
            c = np.delete(c, i-counter, 1)
            d = np.delete(d, i-counter)
            counter += 1
    max_dist = max(d)

    return x, c, max_dist

def coundRt(x, c):
    aver_x = np.mean(x, axis=1, keepdims = True)
    aver_c = np.mean(c, axis=1, keepdims = True)

    tmp_x = x - aver_x
    tmp_c = c - aver_c

    H = np.matmul(tmp_x, tmp_c.T)

    U, S, V = np.linalg.svd(H, full_matrices = True)

    R = np.matmul(V.T, U.T)
    t = aver_c - np.matmul(R, aver_x)

    return R, t