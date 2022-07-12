###############################################################################
# Description
###############################################################################

"""
This class is used to implement a simple fading filter. The filter order can be set to either 1 or 2.
"""



###############################################################################
# Import libraries
###############################################################################

import numpy as np



###############################################################################
# Fading filter class definition
###############################################################################

class FadingFilter:

    # Class constructor
    def __init__(self):

        # Fading filter order
        self.order = 1

        # Fading filter parameter
        self.beta = 0.9

        # Estimated quantities
        self.x = np.array([])
        self.x_dot = np.array([])


    # Filter a new measurement.    
    def filter(self, meas, Ts=float('nan')):

        if self.order == 1:
            # First order fading filter

            if self.x.size == 0:
                # Initialize the filter with the current measurement
                self.x = meas

            else:
                beta = self.beta
                self.x = beta * self.x + (1 - beta) * meas

        elif self.order == 2:
            # Second order fading filter

            if self.x_dot.size == 0:
                # Initialize the filter with the current measurement
                self.x = meas
                self.x_dot = np.zeros(meas.shape)

            else:
                beta = self.beta
                G = 1 - beta**2
                H = (1 - beta)**2

                self.x = self.x + self.x_dot * Ts + G * (meas - (self.x + self.x_dot * Ts))
                self.x_dot = self.x_dot + H / Ts * (meas - (self.x + self.x_dot * Ts))

        return self.x