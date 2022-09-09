import numpy as np
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