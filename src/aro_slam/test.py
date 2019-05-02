from __future__ import absolute_import, division, print_function
import numpy as np
from scipy.spatial import cKDTree
from random import *
import math

def selectRandomSubset(x):
    count = randint(50, x.shape[1])
    x = x.T
    np.random.shuffle(x)
    P = x[0:count]

    return P

# ref must be ref.T !!!, 
def findCorespondences (P, ref, tree, k):
    C = []
    ref = ref.T
    print("Find correspondences")
    for i in P:
        # print("i =")
        # print(i)
        _, idx = tree.query(i, k)
        tmp = ref[idx]
        print("q tmp = ")
        print(tmp)
        for j in tmp:
            tmpList = []
            tmpList.append(i)
            tmpList.append(j)
            C.append(tmpList)
    print("-----------------------")
    print(C)
    return C