import numpy as np

import wheg_utils.central_pattern_generators as cpg
import matplotlib.pyplot as plt


# differential drive math for 4 wheels
A_ = np.array([[1,-1],[-1,-1],[1,-1],[-1,-1]])
def diff_drive(v,w,h,rad,dist):
    A = (1/rad) * np.matmul(A_,[np.array([[1,0],[0,dist/2]])])
    r_dot = np.matmul(A,np.array([v,w]))
    # extension as ratio of radius
    e_tar = np.ones(4)*(h/rad-1)
    return r_dot, e_tar