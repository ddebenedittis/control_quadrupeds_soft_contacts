import numpy as np
from numpy import cos, sin, exp
from numpy.linalg import norm
import quaternion



def quat_rot(v, q):
    """
    Rotate a three dimensional vector v with quaternion q.
    
    params
    ----
    v: Vector to be rotated
    q: Quaternion
    
    return
    ----
    q * [0, v] * q.conj
    """
    
    return quaternion.as_vector_part(
        q * quaternion.from_vector_part(v) * q.conj()
    )
    

def quat_exp(q):
    """
    Compute the quaternion exponential e^q.
    
    params
    ----
    q: quaternion
    
    return
    e^q = e^w ( cos(|v|) + v / |v| sin(|v|) )
    """
    
    v = quaternion.as_vector_part(q)
    
    return exp(q.w) * quaternion.from_float_array(np.concatenate((
        np.array([cos(norm(v))]), v / norm(v) * sin(norm(v))
    )))