import numpy as np



class PlaneEstimator():
    """
    Estimate the contact plane using the feet positions and the list of feet in contact with the terrain. \\
    The plane equation is z = a x + b y + c. The plane coefficients are published in the /state_estimator/terrain topic as {a, b, c}.
    """
    
    def __init__(self) -> None:
        # Save the position of each foot the last time they were in contact with the terrain. These positions are used to compute the plane coefficients.
        self.contact_feet_positions = np.array([
             1.,  1., 0.,   # LF
             1., -1., 0.,   # RF
            -1.,  1., 0.,   # LH
            -1., -1., 0.    # RH
        ])
        
    
    def compute_plane_eq(self) -> np.ndarray:
        # Plane equation: z = a x + b y + c
        
        #     [ x0 y0 1 ]
        # A = [ x1 y1 1 ]
        #     [ ...     ]
        #     [ xn yn 1 ]
        
        A = np.block([
            self.contact_feet_positions.reshape((4, 3))[:, 0:2], np.ones((4,1))
        ])
        
        #     [ z0  ]
        # b = [ z1  ]
        #     [ ... ]
        #     [ zn  ]

        b = self.contact_feet_positions.reshape((4, 3))[:, 2]
        
        # The plane equation is z = a x + b y + c, where self.plane_coeffs = {a, b, c}.
        plane_coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
        
        return plane_coeffs