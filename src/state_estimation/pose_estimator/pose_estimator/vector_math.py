import numpy as np



def skew(x):
    """
    Return the skew-symmetric matrix from a three dimensional vector.
    
    Returns:
        [   0 -v2  v1 ]
        [  v2   0 -v0 ]
        [ -v1  v0   0 ]
    """
    return np.array([
        [    0, -x[2],  x[1]],
        [ x[2],     0, -x[0]],
        [-x[1],  x[0],     0],
    ])