import numpy as np
import pandas as pd


def InverseT(x):
    R = x[:3,:3]
    p = x[:,3][:3]
    RT = R.T
    ip = np.matmul(-1 * RT, p)
    y = np.array([[R.T[0,0], R.T[0,1], R.T[0,2], ip[0]],
                  [R.T[1,0], R.T[1,1], R.T[1,2], ip[1]],
                  [R.T[2,0], R.T[2,1], R.T[2,2], ip[2]],
                  [       0,        0,        0,     1]])
    return y