import numpy as np



class PlaneEstimator():
    """
    Estimate the contact plane using the feet positions and the list of feet in contact with the terrain. \\
    The plane equation is z = a x + b y + c. The plane coefficients are published in the /state_estimator/terrain topic as {a, b, c}.
    """
    
    def __init__(self) -> None:
        # Save the position of each foot the last time they were in contact with the terrain. These positions are used to compute the plane coefficients.
        self.last_contact_feet_position = np.array([
             1.,  1., 0.,
             1., -1., 0.,
            -1.,  1., 0.,
            -1., -1., 0.
        ])
        
    
    def compute_plane_eq(self) -> np.ndarray:
        # Plane equation: z = a x + b y + c
        
        #     [ x0 y0 1 ]
        # A = [ x1 y1 1 ]
        #     [ ...     ]
        #     [ xn yn 1 ]
        
        #     [ z0  ]
        # b = [ z1  ]
        #     [ ... ]
        #     [ zn  ]
        
        A = np.block([
            self.last_contact_feet_position.reshape((4, 3))[:, 0:2], np.ones((4,1))
        ])
        
        b = self.last_contact_feet_position.reshape((4, 3))[:, 2]
        
        # The plane is z = a x + b y + c, where self.plane_coeffs = {a, b, c}.
        plane_coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
        
        return plane_coeffs