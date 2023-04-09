import numpy as np
from numpy import ndarray
import quaternion

from robot_model.robot_model import RobotModel

from .quaternion_math import quat_rot, quat_exp
from .vector_math import skew



# ============================================================================ #
#                                  CONVENTIONS                                 #
# ============================================================================ #

# Quaternion conventions: Hamilton
# See Quaternion kinematics for the error-state Kalman filter
# See https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf

# Real part first
# q = [q_w, q_x, q_y, q_z] = [q_w, q_v]

# Right handed: i j = - j i = k

# Passive: rotate frames

# Local to global
# q = q_nb = rotates a vector from the base frame to the navigation frame.
# R_nb = quaternion.as_rotation_matrix(q) = (qw^2 + |qv|^2) I + 2 qw skew(qv) + 2 qv qv^T



class KalmanFilter:
    """
    Kalman filter for a quadrupedal robot.
    
    Methods:
        update_private_properties
    """
    
    # ======================================================================== #
    #                             CLASS CONSTRUCTOR                            #
    # ======================================================================== #
    
    def __init__(self, robot_name: str):
        
        # ========================= RobotModel Class ========================= #
        
        self._robot_model = RobotModel(robot_name)
        
        # ========================== Time Properties ========================= #
        
        self._previous_time = 0
        self.sample_time = 1/100
                
        # The imu correction is done every decimation_factor samples received by the imu sensor.
        self.decimation_factor = 1
        # When _imu_counter == decimation_factor, the imu correction is performed.
        self._imu_counter = 1
        
        # ========================= Sensor Properties ======================== #
        
        # Gyroscope
        self.gyro_noise = 1e-3                          # [rad^2/s^2]
        self.gyro_drift_noise = 1e-12                   # [(rad/s)^2/s]
        # self.q_sens_gyro = np.quaternion(1, 0, 0, 0)  # orientation of the gyroscope sensor frame wrt body frame
        
        # Accelerometer
        self.acc_noise = 1e-3                           # [(m/s^2)^2]
        self.acc_drift_noise = 1e-12                    # [(m/s^2)^2/s]
        # self.q_sens_acc = np.quaternion(1, 0, 0, 0)   # orientation of the acceleration sensor frame wrt body frame
        
        # Joint encoders
        self.enc_noise = 1e-3                           # [rad^2/s^2]
        self.leg_length = 1                             # [m]
        
        # ===================== Kalman Filter Properties ===================== #
        
        # Position and velocity process noises
        self.pos_noise = 1e-0                           # [m^2/s^2]
        self.vel_noise = 1e-0                           # [(m/s)^2/s]
        
        # Initial process noise P0
        self.init_process_noise = np.diag(np.concatenate((
            1e-10 * np.ones(3),
            1e-10 * np.ones(3),
            1e-10 * np.ones(3),
            1e-10 * np.ones(3),
            1e-10 * np.ones(3),
        )))
        
        # ========================= Earth Parameters ========================= #
        
        self.gravity = 9.81                     # [m/s^2]
        
        # =========================== Filter States ========================== #
        
        # State:
        #         ⌈ quaternion orientation |   ⌈ q   ⌉
        #         | position               |   | p   |
        # state = | velocity               | = | v   | ∈ R^16
        #         | gyroscope bias         |   | ω_b |
        #         | accelerometer bias     |   ⌊ a_b ⌋
        self._state = np.zeros(16)
        self._state[0] = 1  # ! q_w figlio di troia

        # Error state:
        #             ⌈ orientation error        ⌉   ⌈ θ_ε   ⌉
        #             | position error           |   | p_ε   |
        # state_err = | velocity error           | = | v_ε   | ∈ R^15
        #             | gyroscope bias error     |   | ω_b_ε |
        #             ⌊ accelerometer bias error ⌋   ⌊ a_b_ε ⌋
        
        # Angular velocity at the previous time step
        self._ang_vel_old = np.zeros(3)
        
        # Low passed gyroscope measurement. Used for the correction with odometry.
        self._gyro_meas_avg = np.zeros(3)
        
        # Average of the measured acceleration over decimation_factor steps
        self._acc_meas_avg = np.zeros(3)
        
        self._state_covariance = np.zeros((15,15))
        
        self._process_covariance = np.array([])
        
        self._measurement_covariance = np.array([])
        
        self.update_private_properties()

        # ================ Feet Position Correction Parameters =============== #

        # When True, the correction is performed the position of the feet in addition to their velocity.
        self.flag_fuse_odo_pos = True
        
        # Names of the contact feet.
        self._last_contact_feet_names = []
        # Touchdown positions of the feet (by row).
        self._touchdown_positions = np.zeros((4,3))
        
        
        
    # ======================================================================== #
    #                                  GETTERS                                 #
    # ======================================================================== #
    
    @property
    def orientation(self) -> ndarray:
        """
        Orientation of the robot base as a quaternion.\\
        Expresses the rotation from the base frame to the navigation frame (q_nb).

        Returns:
            ndarray: [q_w, q_x, q_y, q_z]]
        """

        return quaternion.from_float_array(self._state[0:4])
    
    @property
    def orientation_as_float_array(self) -> ndarray:
        return self._state[0:4]
    
    @property
    def position(self) -> ndarray:
        """
        Position of the robot base in inertial frame.

        Returns:
            ndarray: [p_x, p_y, p_z]
        """
        
        return self._state[4:7]
    
    @position.setter
    def position(self, p: ndarray):
        if p.size != 3:
            raise ValueError("The position is not a vector of size 3.")
        
        self._state[4:7] = p
    
    @property
    def velocity(self) -> ndarray:
        """
        Velocity of the robot base in inertial frame.

        Returns:
            ndarray: [v_x, v_y, v_z]
        """
        
        return self._state[7:10]
    
    @property
    def gyro_bias(self) -> ndarray:
        return self._state[10:13]
    
    @property
    def acc_bias(self) -> ndarray:
        return self._state[13:16]
    
    
        
    def update_private_properties(self):
        """
        Update the process covariance and the measurement covariance. \\
        Must be performed after changing some changing the filter properties.
        """
        
        dt = self.sample_time
        
        self._process_covariance = np.diag(np.concatenate((
            self.gyro_noise * dt**2 * np.ones(3),
            self.pos_noise * dt**2 * np.ones(3),
            self.vel_noise * dt * np.ones(3),
            self.gyro_drift_noise * dt * np.ones(3),
            self.acc_drift_noise * dt * np.ones(3),
        )))
        
        self._measurement_covariance = np.diag(np.concatenate((
            self.acc_noise * np.ones(3),
            self.enc_noise * self.leg_length * np.ones(1),
            self.enc_noise * self.leg_length * np.ones(1)/20,
        )))
        
        
        
    # ======================================================================== #
    #                                  PREDICT                                 #
    # ======================================================================== #
    
    def predict(self, gyro_meas: ndarray, acc_meas: ndarray, time: float):
        """
        Predict the state and the state covariance using the IMU measurements. \\
        Update the average measured acceleration over (decimation_factor) steps.

        Args:
            gyro_meas (ndarray): gyroscope measurement.
            acc_meas (ndarray): accelerometer measurement.
            time (float): time of the imu measurements.
        """
        
        # Only the nanoseconds are passed.
        self.sample_time = time - self._previous_time
        self._previous_time = time
        if self.sample_time < 0:
            self.sample_time += 1
        
        # Just to be sure, redo this.
        self.update_private_properties()
        
        # Run this code only if new measurements have been received.
        if self.sample_time > 0:
            # Update the state and the state covariance using the IMU measurements.
            self.__state_predict(gyro_meas, acc_meas)
            self.__state_cov_predict(gyro_meas, acc_meas)
            
            # Compute the average acceleration over (decimation_factor) steps.
            # This average acceleration takes into account the rotation of the accelerometer between the time-steps. Used for the correction step, doing this reduces the computational cost and the measurement noise.
            
            gyro_bias = self.gyro_bias
            dt = self.sample_time
            
            # Quaternion representing the rotation during the timestep
            q_rot = quat_exp(1/2*dt*quaternion.from_vector_part(gyro_meas - gyro_bias))
            # Rotate and add the new measured acceleration
            self._acc_meas_avg = quat_rot(q_rot, self._acc_meas_avg) + (acc_meas / self.decimation_factor)
            
            # Perform the imu correction only after decimation_factor timesteps.
            if self._imu_counter < self.decimation_factor:
                self._imu_counter += 1
            else:
                self.__fuse_imu()
                
                # Reset the counter and the average measured acceleration.
                self._imu_counter = 1
                self._acc_meas_avg = np.zeros(3)
      
      
    
    # ======================================================================== #
    #                                 FUSE_IMU                                 #
    # ======================================================================== #
    
    def __fuse_imu(self):
        """
        Fuse the accelerometer measurement to estimate to estimate the orientation from how the gravity is aligned. \n
        The accelerometer is fused every (decimation_factor) steps. This reduces the computational cost and also reduces the accelerometer noise, since the measurement is averaged over multiple steps.
        """
        
        # ========================= Measurement Model ======================== #
        
        q = self.orientation
        acc_bias = self.acc_bias
        
        # Gravity in navigation frame.
        g_n = np.array([0., 0., - self.gravity])
        
        # Gravity in base frame
        g_b = quat_rot(q.conj(), g_n)
        
        # Error model
        z = self._acc_meas_avg - (acc_bias - g_b)
        
        
        # ========================= Observation Model ======================== #
        
        Xdx = self.__X_delta_x(q, 12)
        
        # Derivative of g_b (= R_bn g_n) w.r.t. the orientation q
        dRgdq = self.__dRvdq(q, g_n)
        
        # Measurementt matrix computation
        #x = [     θ_ε;             p_ε;             v_ε;           ω_b_ε;   acc_b_ε,  ]
        Hx = np.block([
             [ - dRgdq, np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3), ],
        ])

        Hk = Hx @ Xdx
        
        # Accelerometer measurement covariance
        R = self._measurement_covariance[0:3,0:3]
        
        
        # ============================ Correction ============================ #
        
        self.__kalman_correct(R, Hk, z)
        
        
        
    # ======================================================================== #
    #                                 FUSE_ODO                                 #
    # ======================================================================== #
    
    def fuse_odo(self, qj, qj_dot, contact_feet_names):
        """
        Fuse the measurements from the robot odometry. \n
        Can use both velocity and position (if flag_fuse_odo_pos is True) measurements of the joints.
        """
        
        if self.sample_time > 0:
        
            # ========================= Measurement Model ======================== #
            
            q = self.orientation
            v = self.velocity
            omega = self._gyro_meas_avg - self.gyro_bias
            
            [B_r_fb, B_v_fb] = self._robot_model.compute_feet_pos_vel(
                np.concatenate((np.zeros(6), np.array([1]), qj)),
                np.concatenate((np.zeros(6), qj_dot)),
                contact_feet_names,
            )
                    
            # Error model
            nc = len(contact_feet_names)    # number of feet in contact with the terrain
            
            R_bn = quaternion.as_rotation_matrix(q).T
            
            z = np.zeros(3 * nc)
            
            # Stack the measurements
            for i in range(nc):
                r_fb = B_r_fb[i, :]
                v_fb = B_v_fb[i, :]
                
                z[3*i:3*i+3] = - v_fb - np.cross(r_fb, omega) - quat_rot(q.conj(), v)
                        
            # ========================= Observation Model ======================== #
            
            Xdx = self.__X_delta_x(q, 12)            
                        
            Hx = np.zeros((3*nc, 16))
            
            for i in range(nc):
                dRvdq = self.__dRvdq(q, v)
                
                # Measurement matrix computation
                # x = [ θ_ε,             p_ε,  v_ε,           ω_b_ε,         acc_b_ε, ].T
                Hx[3*i:3*i+3, :] = np.block([
                    [ dRvdq, np.zeros((3,3)), R_bn, np.zeros((3,3)), np.zeros((3,3)), ]
                ])
            
            Hk = Hx @ Xdx
            
            # Measurement covariance
            R = self._measurement_covariance[3,3] * np.eye(3*nc)
            
            
            # ============================ Correction ============================ #
            
            self.__kalman_correct(R, Hk, z)
            
            
            # ========= Fuse The Information Regarding The Foot Positions ======== #
            
            if self.flag_fuse_odo_pos:
                self.__fuse_odo_pos(contact_feet_names, B_r_fb)



    # ======================================================================== #
    #                               FUSE_ODO_POS                               #
    # ======================================================================== #
    
    def __fuse_odo_pos(self, contact_feet_names, B_r_fb):
        """
        Fuse the measurements from the robot odometry. In particular, fuse only the information regarding the relative position.
        """
        
        # ========================= Measurement Model ======================== #
        
        q = self.orientation
        p = self.position
                
        R_nb = quaternion.as_rotation_matrix(q)
        
        # Error model
        nc = len(contact_feet_names)    # number of feet in contact with the terrain
        
        z = np.zeros(3 * nc)
        
        self.__update_touchdown_positions(contact_feet_names, B_r_fb, R_nb)
        
        # Stack the measurements
        for i in range(nc):
            touchdown_pos = self._touchdown_positions[i,:]

            r = B_r_fb[i, :]
            
            z[3*i:3*i+3] = touchdown_pos - (p + R_nb @ r)
        
        
        # ========================= Observation Model ======================== #
        
        Xdx = self.__X_delta_x(q, 12)
               
        Hx = np.zeros((3*nc, 16))
        
        for i in range(nc):
            r = B_r_fb[i, :]
            
            dRrdq = self.__dRvdq(q.conj(), r)
            
            # Measurement matrix computation
            # x = [    θ_ε,       p_ε,             v_ε,           ω_b_ε,         acc_b_ε, ].T
            Hx[3*i:3*i+3, :] = np.block([
                 [ - dRrdq, np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), ]
            ])
        
        Hk = Hx @ Xdx
        
        # Measurement covariance
        R = self._measurement_covariance[4,4] * np.eye(3*nc)
        
        
        # ============================ Correction ============================ #
        
        self.__kalman_correct(R, Hk, z)


    
    # ======================================================================== #
    #                        UPDATE_TOUCHDOWN_POSITIONS                        #
    # ======================================================================== #

    def __update_touchdown_positions(self, contact_feet_names, B_r_fb, R_nb):
        """
        
        """
        if contact_feet_names == self._last_contact_feet_names:
            pass
        else:
            new_touchdown_positions = np.zeros((len(contact_feet_names), 3))
            
            for i in range(len(contact_feet_names)):
                if contact_feet_names[i] in self._last_contact_feet_names:
                    # This foot was already in contact with the terrain
                    new_touchdown_positions[i,:] = self._touchdown_positions[
                        self._last_contact_feet_names.index(contact_feet_names[i]),:]
                else:
                    # This foot just entered in contact with the terrain
                    new_touchdown_positions[i,:] = R_nb @ B_r_fb[i,:] + self.position
                    new_touchdown_positions[i,2] = 0
                    
            self._last_contact_feet_names = contact_feet_names
            self._touchdown_positions = new_touchdown_positions
        
    
    
    # ======================================================================== #
    #                               STATE_PREDICT                              #
    # ======================================================================== #
    
    def __state_predict(self, gyro_meas, acc_meas):
        """
        Compute the state after a time-step using the received IMU measurements.
        """
        
        # ================== Get All The Relevant Quantities ================= #
                
        dt = self.sample_time
        
        # States
        q = self.orientation
        p = self.position
        v = self.velocity
        w_b = self.gyro_bias
        acc_b = self.acc_bias
        
        
        # Obtain the new, old and average angular velocities. They are used for integrating the quaternion.
        w_new = gyro_meas - w_b
        w_old = self._ang_vel_old
        w_avg = 1/2 * (w_new + w_old)
        
        self._gyro_meas_avg = self._gyro_meas_avg*0.75 + gyro_meas*0.25
        
        # Gravity
        g = np.array([0, 0, - self.gravity])
        
        
        # ======================= Integrate The States ======================= #
        
        q = q * (quaternion.from_rotation_vector(w_avg * dt) + dt**2/24 * quaternion.from_vector_part(np.cross(w_old, w_new)))
        q = np.normalized(q)    # better be safe than sorry, let's do this too :)
        
        R_nb = quaternion.as_rotation_matrix(q)
        
        p += v*dt + 1/2 * (R_nb @ (acc_meas - acc_b) + g) * dt**2
        v += (R_nb @ (acc_meas - acc_b) + g) * dt
        
        
        # ============= Update The State And The Angular Velocity ============ #
        
        self._state = np.concatenate((
            quaternion.as_float_array(q),
            p,
            v,
            w_b,
            acc_b,
        ))
        
        self._ang_vel_old = w_new
        
        
    
    # ======================================================================== #
    #                             STATE_COV_PREDICT                            #
    # ======================================================================== #
    
    def __state_cov_predict(self, gyro_meas, acc_meas):
        """
        Compute the state covariance after a time-step using the received IMU measurements.
        """
        
        # ================== Get All The Relevant Quantities ================= #
        
        dt = self.sample_time

        # gyro_bias = self.gyro_bias
        acc_bias = self.acc_bias
        
        R = quaternion.as_rotation_matrix(self.orientation)
        
        
        # ============= Compute The Error State Transition Matrix ============ #
        
        # This version is slightly more accurate (u sure?) but has a higher computational cost.
        # F_err = np.block([
        #     [ np.eye(3) - skew(gyro_meas - gyro_bias)*dt,  np.zeros((3,3)),  np.zeros((3,3)),            -R*dt,  np.zeros((3,3)), ],
        #     [     -1/2*R*skew(acc_meas - acc_bias)*dt**2,        np.eye(3),     np.eye(3)*dt,  np.zeros((3,3)),      1/2*R*dt**2, ],
        #     [            -R*skew(acc_meas - acc_bias)*dt,  np.zeros((3,3)),        np.eye(3),  np.zeros((3,3)),             R*dt, ],
        #     [                            np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),        np.eye(3),  np.zeros((3,3)), ],
        #     [                            np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),        np.eye(3), ],
        # ])
        
        F_err = np.block([
            [                           np.eye(3),  np.zeros((3,3)),  np.zeros((3,3)),            -R*dt,  np.zeros((3,3)), ],
            [                     np.zeros((3,3)),        np.eye(3),     np.eye(3)*dt,  np.zeros((3,3)),  np.zeros((3,3)), ],
            [ -skew(R @ (acc_meas - acc_bias))*dt,  np.zeros((3,3)),        np.eye(3),  np.zeros((3,3)),            -R*dt, ],
            [                     np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),        np.eye(3),  np.zeros((3,3)), ],
            [                     np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),        np.eye(3), ],
        ])
        
        
        # ==================== Update The State Covariance =================== #
        
        self._state_covariance = F_err @ self._state_covariance @ F_err.T + self._process_covariance
        
    
    
    # ======================================================================== #
    #                              KALMAN_CORRECT                              #
    # ======================================================================== #
    
    def __kalman_correct(self, R, Hk, z):
        """
        Perform some steps in common for every kalman correction step: compute the Kalman equations, compute x_err and P_cor, and update the state and the state covariance.
        """
        
        # ========================= Kalman Equations ========================= #
        
        [x_err, P_cor] = self.__kalman_eq(self._state_covariance, R, Hk, z)

        Gk = np.block([
            [np.eye(3) + skew(1/2*x_err[0:3]), np.zeros((3,12))],
            [                np.zeros((12,3)),       np.eye(12)],
        ])
        
        P_cor = Gk @ P_cor @ Gk.T
        
        
        # ===== Correct The State And Update The State Covariance Matrix ===== #
        
        self.__correct(x_err)
        
        self._state_covariance = P_cor
    
    
    
    # ======================================================================== #
    #                                  CORRECT                                 #
    # ======================================================================== #
    
    def __correct(self, state_err: ndarray):
        """
        Correct the filter state using the error state:\\
        state_corr = state ⊕ state_err
        """
        
        # Sum the rotation vector error to the orientation quaternion
        self._state[0:4] = quaternion.as_float_array(
            quaternion.from_rotation_vector(state_err[0:3]) * self.orientation
        )
        
        self._state[4:] += state_err[3:]
    
    
    
    # ========================================================================= #
    #                               STATIC METHODS                              #
    # ========================================================================= #
    
    @staticmethod
    def __kalman_eq(P, R, H, z) -> tuple[ndarray, ndarray]:
        """
        Compute the Kalman equations.
        
        Returns:
            x_err (ndarray): 
            P (ndarray): 
        """
        
        # Innovation covariance
        S = R + H @ P @ H.T
        
        # Kalman gain
        K = P @ H.T @ np.linalg.inv(S)
        
        x_err = K @ z
        
        # Error estimate covariance update
        # P = P - K @ H @ P     # simple form
        # P = P - K @ S @ K.T   # symmetric form
        
        # Joseph form (symmetric and positive definite)
        I = np.eye(P.shape[0])
        P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T
        
        return [x_err, P]
    
    
    @staticmethod
    def __Q_delta_theta(q: ndarray) -> ndarray:
        qw = q.w; qx = q.x; qy = q.y; qz = q.z
        
        Q_delta_theta = 1/2 * np.array([
            [-qx, -qy, -qz,],
            [ qw,  qz, -qy,],
            [-qz,  qw,  qx,],
            [ qy, -qx,  qw,],
        ])
        
        return Q_delta_theta
    
    
    @staticmethod
    def __X_delta_x(q: ndarray, n: int) -> ndarray:
        """
        Computes the partial derivative of the state w.r.t. the error state. \\
        X_dx = ∂x / ∂x_ε
        
        Args:
            q (quaternion): orientation q_nb.
            n (int): remaining dimension.
            
        Returns:
            ndarray: size = [4+n, 3+n]
        """
        
        Q_delta_theta = KalmanFilter.__Q_delta_theta(q)
        
        Xdx = np.block([
            [   Q_delta_theta, np.zeros((4,n)) ],
            [ np.zeros((n,3)),       np.eye(n) ],
        ])
        
        return Xdx
    
    
    @staticmethod
    def __dRvdq(q, v):
        """
        Computes d/dq (R(q) @ v),
        where R(q) = quaternion.as_rotation_matrix(q)
        """
                
        qw = q.w; qx = q.x; qy = q.y; qz = q.z
        vx = v[0]; vy = v[1]; vz = v[2]
        
        return np.array([
            [2*qw*vx - 2*qy*vz + 2*qz*vy, 2*qx*vx + 2*qy*vy + 2*qz*vz, 2*qx*vy - 2*qw*vz - 2*qy*vx, 2*qw*vy + 2*qx*vz - 2*qz*vx],
            [2*qw*vy + 2*qx*vz - 2*qz*vx, 2*qw*vz - 2*qx*vy + 2*qy*vx, 2*qx*vx + 2*qy*vy + 2*qz*vz, 2*qy*vz - 2*qw*vx - 2*qz*vy],
            [2*qw*vz - 2*qx*vy + 2*qy*vx, 2*qz*vx - 2*qx*vz - 2*qw*vy, 2*qw*vx - 2*qy*vz + 2*qz*vy, 2*qx*vx + 2*qy*vy + 2*qz*vz],
        ])