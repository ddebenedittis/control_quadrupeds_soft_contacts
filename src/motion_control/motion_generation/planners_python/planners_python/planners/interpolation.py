from enum import Enum
from math import pi
import numpy as np



class InterpolationMethod(Enum):
    spline_5th = 1
    spline_3rd = 2
    cycloid = 3



class Interpolation:
    def __init__(self):
        self.method = InterpolationMethod.spline_3rd
        
        # Time duration of a step swing (both upwards and downwards movement).
        self.Ts = 0.2
        
        # Phase delay before which the foot starts moving in the horizontal direction.
        self.horizontal_delay = 0.1
        
        self.step_height = 0.1
        
        # Positive when the foot penetrates the terrain.
        self.foot_penetration = - 0.025
        
        
    # ============================== Interpolate ============================= #
        
    def interpolate(self, pi, pf, phi):
        if self.method == InterpolationMethod.spline_3rd or self.method == InterpolationMethod.spline_5th:
            return self._foot_trajectory_spline_2(pi, pf, phi)
        elif self.method == InterpolationMethod.cycloid:
            d = np.linalg.norm(pf - pi)
            theta = np.arctan2( pf[1] - pi[1], pf[0] - pi[0] )
            return self._foot_trajectory_cycloid(pi, d, theta, phi)
        
        
    # ============================ _compute_phi_m ============================ #
    
    def _compute_phi_m(self, phi):
        """Compute the modified phi that takes into account the horizontal delay"""
        
        phi_m = (phi - self.horizontal_delay/2) / (1 - self.horizontal_delay)
        return min(max(0, phi_m), 1)


    # ================================ _spline =============================== #
    
    # Spline from pi to pf with parameter phi in [0,1]
    # With a fifth order spline it is possible to set both the velocity and the acceleration at the start and at the end of the spline to zero.
    # With a third order spline it is possible to set only the velocity of the point to zero at the start and at the end of the spline (in addition to setting the initial and final spline points).
    def _spline(self, pi, pf, phi):
        
        # phi /in [0, 1]
        
        # Local time of transition phase and its derivatives
        if self.method == InterpolationMethod.spline_5th:

            f_t = phi**3 * (10 - 15 * phi + 6 * phi**2)

            f_t_dot = 30 * phi**2 - 60 * phi**3 + 30 * phi**4

            f_t_ddot = 60 * phi - 180 * phi**2 + 120 * phi**3

        elif self.method == InterpolationMethod.spline_3rd:

            f_t = phi**2 * (3 - 2*phi)

            f_t_dot = 6*phi - 6*phi**2

            f_t_ddot = 6 - 12*phi


        # Polynomial spline and its derivatives
        p_t = (1 - f_t) * pi + f_t * pf

        v_t = - f_t_dot * pi + f_t_dot * pf

        a_t = - f_t_ddot * pi + f_t_ddot * pf
        

        return p_t, v_t, a_t
    
    
    # ======================== _foot_trajectory_spline ======================= #
    
    def _foot_trajectory_spline(self, p_i, d, theta, phi):
        """
        Compute the foot position, velocity, and acceleration using splines.

        Args:
            p_i (_type_): initial position
            phi (_type_): phase \in [0, 1]
            theta (_type_): angle between the desired foot displacement and the heading direction

        Returns:
            _type_: _description_
        """
        
        if phi <= 0.5:
            phi_2 = phi * 2
            p_z, v_z, a_z = self._spline(0, self.step_height, phi_2)
        else:
            phi_2 = 2 * phi - 1
            p_z, v_z, a_z = self._spline(self.step_height, - self.foot_penetration, phi_2)
        
        #! The robot walks faster when the velocity and the acceleration are not scaled.
        # The velocity and acceleration must be scaled to take into account the time in which the swing is performed (since phi is a normalized quantity).
        v_z = v_z / (self.Ts / 2)
        a_z = a_z / (self.Ts / 2)
        
        p_tan, v_tan, a_tan = self._spline(0, d, self._compute_phi_m(phi))
        #! The robot walks faster when the velocity and the acceleration are not scaled.
        v_tan = v_tan / self.Ts
        a_tan = a_tan / self.Ts
        
        p_t = p_i + np.array([p_tan * np.cos(theta), p_tan * np.sin(theta), p_z])
        v_t = np.array([v_tan * np.cos(theta), v_tan * np.sin(theta), v_z])
        a_t = np.array([a_tan * np.cos(theta), a_tan * np.sin(theta), a_z])
        
        return p_t, v_t, a_t
    
    
    # ======================= _foot_trajectory_spline_2 ====================== #
    
    def _foot_trajectory_spline_2(self, p_i, p_f, phi):
        """
        Compute the foot position, velocity, and acceleration using splines.

        Args:
            p_i (_type_): _description_
            p_f (_type_): _description_
            phi (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        if phi <= 0.5:
            phi_2 = phi * 2
            p_z, v_z, a_z = self._spline(0, self.step_height, phi_2)
        else:
            phi_2 = 2 * phi - 1
            p_z, v_z, a_z = self._spline(self.step_height, - self.foot_penetration, phi_2)
        
        #! The robot walks faster when the velocity and the acceleration are not scaled.
        # The velocity and acceleration must be scaled to take into account the time in which the swing is performed (since phi is a normalized quantity).
        v_z = v_z / (self.Ts / 2)
        a_z = a_z / (self.Ts / 2)
        
        p_xy, v_xy, a_xy = self._spline(p_i[0:2], p_f[0:2], self._compute_phi_m(phi))
        #! The robot walks faster when the velocity and the acceleration are not scaled.
        v_xy = v_xy / self.Ts
        a_xy = a_xy / self.Ts
        
        p_t = np.concatenate((p_xy, np.array([p_z])))
        v_t = np.concatenate((v_xy, np.array([v_z])))
        a_t = np.concatenate((a_xy, np.array([a_z])))
        
        return p_t, v_t, a_t
    
    
    # ======================= _foot_trajectory_cycloid ======================= #
    
    def _foot_trajectory_cycloid(self, p_i, d, theta, phi):
        """
        Compute the foot position, velocity, and acceleration using a cycloid.

        Args:
            p_i (_type_): initial position
            phi (_type_): phase \in [0, 1]
            theta (_type_): angle between the desired foot displacement and the heading direction

        Returns:
            _type_: _description_
        """
        
        h = self.step_height
        T = self.Ts
        
        t = phi * T
        
        # Compute the horizontal displacement, velocity, and acceleration
        phi_m = self._compute_phi_m(phi)
        t_m = phi_m * T
        
        x = d * (t_m / T - 1 / (2 * pi) * np.sin(2 * pi * t_m / T))
        x_dot = d / T * (1 - np.cos(2 * pi * t_m / T))
        x_ddot = 2 * pi * d / T**2 * np.sin(2 * pi * t_m / T)

        # Compute the vertical position, velocity and acceleration.
        z = 2 * h * (t/T - 1 / (4*pi) * np.sin(4 * pi * t / T))
        z_dot = 2 * h / T * (1 - np.cos(4 * pi * t / T))
        z_ddot = 2 * h / T**2 * 4 * pi * np.sin(4 * pi * t / T)
        
        if (t > T/2):
            z = 2 * h - z
            z_dot = - z_dot
            z_ddot = - z_ddot
            
        p_t = p_i + np.array([x * np.cos(theta), x * np.sin(theta), z])
        v_t = np.array([x_dot * np.cos(theta), x_dot * np.sin(theta), z_dot])
        a_t = np.array([x_ddot * np.cos(theta), x_ddot * np.sin(theta), z_ddot])
        
        return p_t, v_t, a_t