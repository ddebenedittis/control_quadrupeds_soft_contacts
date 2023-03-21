import numpy as np



class FadingFilter:
    """
    Simple fading filter class. The fading filter order can be either one or two.
    """

    # Class constructor
    def __init__(self):

        # Fading filter order
        self._order = 1

        # Fading filter parameter. x_new = beta*x_old + (1-beta)*meas
        self._beta = 0.9

        # Estimated quantities
        self.x = np.array([])
        self.x_dot = np.array([])
        
    
    @property
    def order(self):
        """The filter order"""
        return self._order
    
    @order.setter
    def order(self, value: int):
        if value == 1 or value == 2:
            self._order = value
        else:
            raise ValueError("The filter order must be either 1 or 2")
        
        
    @property
    def beta(self):
        """The filter beta coefficient. x_new = beta*x_old + (1-beta)*meas"""
        return self._beta
    
    @beta.setter
    def beta(self, value):
        if value <= 1 and value >= 0:
            self._beta = value
        else:
            raise ValueError("The filter beta coefficient must be between 0 and 1")


    # Filter a new measurement.    
    def filter(self, meas, Ts=float('nan')):

        if self._order == 1:
            # First order fading filter

            if self.x.size == 0:
                # Initialize the filter with the current measurement
                self.x = meas

            else:
                beta = self._beta
                self.x = beta * self.x + (1 - beta) * meas

        elif self._order == 2:
            # Second order fading filter

            if self.x_dot.size == 0:
                # Initialize the filter with the current measurement
                self.x = meas
                self.x_dot = np.zeros(meas.shape)

            else:
                beta = self._beta
                G = 1 - beta**2
                H = (1 - beta)**2

                self.x = self.x + self.x_dot * Ts + G * (meas - (self.x + self.x_dot * Ts))
                self.x_dot = self.x_dot + H / Ts * (meas - (self.x + self.x_dot * Ts))

        return self.x