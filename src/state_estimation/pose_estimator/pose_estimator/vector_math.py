import numpy as np



def skew(x):
    """
    Return the skew-symmetric matrix from a three dimensional vector.
    """
    return np.array([[    0, -x[2],  x[1]],
                     [ x[2],     0, -x[0]],
                     [-x[1],  x[0],    0]])