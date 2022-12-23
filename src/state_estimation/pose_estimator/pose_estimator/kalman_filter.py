import numpy as np
import quaternion

from .vector_math import skew
from .quaternion_math import quat_rot, quat_exp
from robot_model.robot_model import RobotModel



class KalmanFilter:
    """
    Implements a kalman filter for a quadrupedal robot.
    """
    
    # ======================================================================== #
    #                             CLASS CONSTRUCTOR                            #
    # ======================================================================== #
    
    def __init__(self, robot_name):
        
        # ========================= RobotModel Class ========================= #
        
        self._robot_model = RobotModel(robot_name)
        
        # ========================== Time Properties ========================= #
        
        self.previous_time = 0
        self.sample_time = 1/100
                
        self.decimation_factor = 1   # Number of IMU samples joined together to reduce the computational cost
        self._imu_counter = 0
        
        # ========================= Sensor Properties ======================== #
        
        # Gyroscope
        self.gyro_noise = 1e-3                       # [rad^2/s^2]
        self.gyro_drift_noise = 1e-6                 # [(rad/s)^2/s]
        # self.q_sens_gyro = np.quaternion(1, 0, 0, 0) # orientation of the gyroscope sensor frame wrt body frame
        
        # Accelerometer
        self.acc_noise = 1e-3;                       # [(m/s^2)^2]
        self.acc_drift_noise = 1e-9                  # [(m/s^2)^2/s]
        # self.q_sens_acc = np.quaternion(1, 0, 0, 0)  # orientation of the acceleration sensor frame wrt body frame
        
        # Joint encoders
        self.enc_noise = 1e-3                        # [rad^2/s^2]
        self.leg_length = 1                          # [m]
        
        # ===================== Kalman Filter Properties ===================== #
        
        # Position and velocity process noises
        self.pos_noise = 1e-0;                       # [m^2/s^2]
        self.vel_noise = 1e-0;                       # [(m/s)^2/s]
        
        # Initial process noise P0
        self.init_process_noise = np.diag(np.concatenate((
            1e-10 * np.ones(3),
            1e-0 * np.ones(3),
            1e-0 * np.ones(3),
            1e-10 * np.ones(3),
            1e-10 * np.ones(3)
        )))
        
        # ========================= Earth Parameters ========================= #
        
        self.gravity = 9.81                     # [m/s^2]
        
        # =========================== Filter States ========================== #
        
        # State:
        #         ⌈ quaternion orientation |   ⌈ q   ⌉
        #         | gyroscope bias         |   | ω_b |
        # state = | accelerometer bias     | = | a_b | ∈ R^16
        #         | position               |   | p   |
        #         | velocity               |   ⌊ v   ⌋
        self._state = np.zeros(16)
        self._state[0] = 1  # ! q_w figlio di troia
        
        # Error state:
        #             ⌈ orientation error          ⌉   ⌈ θ_ε   ⌉
        #             | gyroscope bias error       |   | ω_b_ε |
        # state_err = | accelerometer bias error   | = | a_b_ε | ∈ R^15
        #             | position error             |   | p_ε   |
        #             ⌊ velocity error             ⌋   ⌊ v_ε   ⌋
        
        # Angular velocity at the previous time step
        self._ang_vel_old = np.zeros(3)
        
        # Average of the measured acceleration over decimation_factor steps
        self._acc_meas_avg = np.zeros(3)
        
        # Low passed gyroscope measurement. Used for the correction with odometry.
        self._gyro_meas_avg = np.zeros(3)
        
        self._state_covariance = np.zeros((15,15))
        
        self._process_covariance = np.array([])
        
        self._measurement_covariance = np.array([])
        
        self.update_private_properties()

        # ================ Feet Position Correction Parameters =============== #

        self._flag_fuse_odo_pos = True
        
        self._last_contact_feet_names = []
        self._touchdown_positions = np.zeros((4,3))
        
        
        
    def update_private_properties(self):
        """
        Update the process covariance and the measurement covariance after changing some filter properties.
        """
        
        dt = self.sample_time
        
        self._process_covariance = np.diag(np.concatenate((
            self.gyro_noise * dt**2 * np.ones(3),
            self.gyro_drift_noise * dt * np.ones(3),
            self.acc_drift_noise * dt * np.ones(3),
            self.pos_noise * dt**2 * np.ones(3),
            self.vel_noise * dt * np.ones(3)
        )))
        
        self._measurement_covariance = np.diag(np.concatenate((
            self.acc_noise * np.ones(3),
            self.enc_noise * self.leg_length * np.ones(1),
            self.enc_noise * self.leg_length * np.ones(1)/20
        )))
        
        
        
    # ======================================================================== #
    #                                  PREDICT                                 #
    # ======================================================================== #
    
    def predict(self, gyro_meas, acc_meas, time):
        """
        Predict the state and the state covariance by using the IMU measurements.
        Update the average of measured acceleration over (decimation_factor) steps.
        """
        
        #
        self.sample_time = time - self.previous_time
        self.previous_time = time
        if self.sample_time < 0:
            self.sample_time += 1
        
        self.update_private_properties()
        
        if self.sample_time > 0:
            # Update the state and the state covariance using the IMU measurements.
            self.__state_predict(gyro_meas, acc_meas)
            self.__state_cov_predict(gyro_meas, acc_meas)
            
            # Compute the average acceleration over (decimation_factor) steps. This average acceleration takes into account the rotation of the accelerometer between the time-steps. Used for the correction step, doing this reduces the computational cost and the measurement noise.
            gyro_bias = self._state[4:7]
            dt = self.sample_time
            # R = quaternion.as_rotation_matrix(np.exp(1/2*dt * quaternion.from_vector_part(gyro_meas - gyro_bias)))
            
            # self._acc_meas_avg = R.T @ self._acc_meas_avg + (acc_meas / self.decimation_factor)
            self._acc_meas_avg = quat_rot(self._acc_meas_avg, quat_exp(1/2*dt*quaternion.from_vector_part(gyro_meas - gyro_bias))) + (acc_meas / self.decimation_factor)
            
            if self._imu_counter < self.decimation_factor:
                self._imu_counter += 1
            else:
                self.__fuse_imu()
                self._imu_counter = 0
                self._acc_meas_avg = np.zeros(3)
      
      
    
    # ======================================================================== #
    #                                 FUSE_IMU                                 #
    # ======================================================================== #
    
    def __fuse_imu(self):
        """
        Fuse the accelerometer measurement to estimate to estimate the orientation from how the gravity is aligned.
        The accelerometer is fused every (decimation_factor) steps. This reduces the computational cost and also reduces the accelerometer noise, since the measurement is averaged over multiple steps.
        """
        
        # ========================= Measurement Model ======================== #
        
        q = quaternion.from_float_array(self._state[0:4])
        acc_bias = self._state[7:10]
        acc_meas_avg = self._acc_meas_avg
        g_n = np.array([0, 0, - self.gravity])    # gravity in navigation frame
        
        # Reset the average measured acceleration to zero
        self._acc_meas_avg = np.zeros(3)
        
        # R_bn = quaternion.as_rotation_matrix(q).T = rotation matrix from n to b 
        g_b = quat_rot(g_n, q.conj())
        
        # Error model
        z = acc_meas_avg - (acc_bias - g_b)
        
        
        # ========================= Observation Model ======================== #
        
        Xdx = self.__X_delta_x(q, 12)
        
        dRgdq = self.gravity * np.block([
            [-2*q.y,  2*q.z, -2*q.w, 2*q.x],
            [ 2*q.x,  2*q.w,  2*q.z, 2*q.y],
            [ 2*q.w, -2*q.x, -2*q.y, 2*q.z]
        ])
        
        # Measurementt matrix computation
        #x = [   θ_ε,           ω_b_ε,    acc_b_ε,             p_ε,           v_ε ].T
        Hx = np.block([
             [ dRgdq, np.zeros((3,3)),  np.eye(3), np.zeros((3,3)), np.zeros((3,3))]
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
        Fuse the measurements from the robot odometry.
        """
        
        if self.sample_time > 0:
        
            # ========================= Measurement Model ======================== #
            
            q = quaternion.from_float_array(self._state[0:4])
            gyro_bias = self._state[4:7]
            v = self._state[13:16]
            
            [B_r_fb, B_v_fb] = self._robot_model.compute_feet_pos_vel(qj, qj_dot, contact_feet_names)
                    
            # Error model
            nc = len(contact_feet_names)    # number of feet in contact with the terrain
            
            R_bn = quaternion.as_rotation_matrix(q).T
            
            z = np.zeros(3 * nc)
            
            # Stack the measurements
            for i in range(nc):
                r_fb = B_r_fb[i, :]
                v_fb = B_v_fb[i, :]
                
                z[3*i:3*i+3] = - v_fb - np.cross((self._gyro_meas_avg - gyro_bias), r_fb) - quat_rot(v, q.conj())
                        
            # ========================= Observation Model ======================== #
            
            Xdx = self.__X_delta_x(q, 12)
            
            qw = q.w; qx = q.x; qy = q.y; qz = q.z
            
            Hx = np.zeros((3*nc, 16))
            
            for i in range(nc):
                r = B_r_fb[i, :]
                
                rx = r[0]; ry = r[1]; rz = r[2]
                
                # d/dq (R * B_r_fb)
                dRrdq = np.array([
                    [2*rx*qw + 2*ry*qz - 2*rz*qy, 2*rx*qx + 2*ry*qy + 2*rz*qz, 2*ry*qx - 2*rx*qy - 2*rz*qw, 2*ry*qw - 2*rx*qz + 2*rz*qx],
                    [2*ry*qw - 2*rx*qz + 2*rz*qx, 2*rx*qy - 2*ry*qx + 2*rz*qw, 2*rx*qx + 2*ry*qy + 2*rz*qz, 2*rz*qy - 2*ry*qz - 2*rx*qw],
                    [2*rx*qy - 2*ry*qx + 2*rz*qw, 2*rx*qz - 2*ry*qw - 2*rz*qx, 2*rx*qw + 2*ry*qz - 2*rz*qy, 2*rx*qx + 2*ry*qy + 2*rz*qz]
                ])
                
                # Measurement matrix computation
                #x = [   θ_ε,           ω_b_ε,         acc_b_ε,             p_ε,  v_ε ].T
                Hx[3*i:3*i+3, :] = np.block([
                    [ -dRrdq, np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), R_bn ]
                ])
            
            Hk = Hx @ Xdx
            
            # Measurement covariance
            R = self._measurement_covariance[3,3] * np.eye(3*nc)
            
            
            # ============================ Correction ============================ #
            
            self.__kalman_correct(R, Hk, z)
            
            
            # ========= Fuse The Information Regarding The Foot Positions ======== #
            
            if self._flag_fuse_odo_pos == True:
                self.__fuse_odo_pos(contact_feet_names, B_r_fb)



    # ======================================================================== #
    #                               FUSE_ODO_POS                               #
    # ======================================================================== #
    
    def __fuse_odo_pos(self, contact_feet_names, B_r_fb):
        """
        Fuse the measurements from the robot odometry. In particular, fuse only the information regarding the relative position.
        """
        
        # ========================= Measurement Model ======================== #
        
        q = quaternion.from_float_array(self._state[0:4])
        p = self._state[10:13]
                
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
        
        # R = R_b^n = R_nb = R_bn.T, thus we take the conjugate of the quaternion
        qw = q.w; qx = - q.x; qy = - q.y; qz = - q.z
        
        Hx = np.zeros((3*nc, 16))
        
        for i in range(nc):
            r = B_r_fb[i, :]
            
            rx = r[0]; ry = r[1]; rz = r[2]
            
            # d/dq (- R_nb * B_r_fb)
            dRrdq = np.array([
                [2*rx*qw + 2*ry*qz - 2*rz*qy, 2*rx*qx + 2*ry*qy + 2*rz*qz, 2*ry*qx - 2*rx*qy - 2*rz*qw, 2*ry*qw - 2*rx*qz + 2*rz*qx],
                [2*ry*qw - 2*rx*qz + 2*rz*qx, 2*rx*qy - 2*ry*qx + 2*rz*qw, 2*rx*qx + 2*ry*qy + 2*rz*qz, 2*rz*qy - 2*ry*qz - 2*rx*qw],
                [2*rx*qy - 2*ry*qx + 2*rz*qw, 2*rx*qz - 2*ry*qw - 2*rz*qx, 2*rx*qw + 2*ry*qz - 2*rz*qy, 2*rx*qx + 2*ry*qy + 2*rz*qz]
            ])
            
            # Measurement matrix computation
            #x = [    θ_ε,           ω_b_ε,         acc_b_ε,       p_ε,  v_ε ].T
            Hx[3*i:3*i+3, :] = np.block([
                 [ -dRrdq, np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3)) ]
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
                    new_touchdown_positions[i,:] = R_nb @ B_r_fb[i,:] + self._state[10:13]
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
        q = quaternion.from_float_array(self._state[0:4])
        w_b = self._state[4:7]
        acc_b = self._state[7:10]
        p = self._state[10:13]
        v = self._state[13:16]
        
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
        
        p += v*dt + 1/2*dt**2 * (R_nb @ (acc_meas - acc_b) + g)
        v += dt * (R_nb @ (acc_meas - acc_b) + g)
        
        
        # ============= Update The State And The Angular Velocity ============ #
        
        self._state = np.concatenate((quaternion.as_float_array(q), w_b, acc_b, p, v))
        
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

        gyro_bias = self._state[4:7]        
        acc_bias = self._state[7:10]
        
        R = quaternion.as_rotation_matrix(quaternion.from_float_array(self._state[0:4]))
        
        
        # ============= Compute The Error State Transition Matrix ============ #
        
        F_err = np.block([
            [np.eye(3) - dt*skew(gyro_meas - gyro_bias),        -dt*np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [                           np.zeros((3,3)),            np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
            [                           np.zeros((3,3)),      np.zeros((3,3)),       np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
            [         -1/2*dt**2*R*skew(acc_meas + acc_bias), np.zeros((3,3)),     1/2*dt**2*R,       np.eye(3),    dt*np.eye(3)],
            [                -dt*R*skew(acc_meas + acc_bias), np.zeros((3,3)),            dt*R, np.zeros((3,3)),       np.eye(3)]
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
            [np.eye(3) - skew(1/2*x_err[0:3]), np.zeros((3,12))],
            [                np.zeros((12,3)),       np.eye(12)]
        ])
        
        P_cor = Gk @ P_cor @ Gk.T
        
        
        # ===== Correct The State And Update The State Covariance Matrix ===== #
        
        self.__correct(x_err)
        
        self._state_covariance = P_cor
    
    
    
    # ======================================================================== #
    #                                  CORRECT                                 #
    # ======================================================================== #
    
    def __correct(self, state_err):
        """
        Correct the filter state using the error state:
            state_corr = state ⊕ state_err
        """
        
        # Sum the rotation vector error to the orientation quaternion
        self._state[0:4] = quaternion.as_float_array(
            quaternion.from_float_array(self._state[0:4]) * quaternion.from_rotation_vector(state_err[0:3])
        )
        
        self._state[4:] += state_err[3:]
    
    
    
    # ========================================================================= #
    #                               STATIC METHODS                              #
    # ========================================================================= #
    
    @staticmethod
    def __kalman_eq(P, R, H, z):
        """
        Compute the Kalman equations. \n
        return [x_err, P]
        """
        
        # Innovation covariance
        S = R + H @ P @ H.T
        
        # Kalman gain
        K = P @ H.T @ np.linalg.inv(S)
        
        # Error estimate covariance update
        P = P - K @ H @ P
        
        x_err = K @ z
        
        return [x_err, P]
    
    
    @staticmethod
    def __X_delta_x(q, n):
        """
        Computes X_dx. Necessary because quaternions are used.
        """
        
        # Quaternion parts
        qw = q.w
        qv = quaternion.as_vector_part(q)
        
        # Quaternion matrix form (NOT a rotation matrix)
        q_matrix = np.block([
            [               qw,                     -qv],
            [qv.reshape((3,1)), qw*np.eye(3) + skew(qv)]
        ])
        
        Q_delta_theta = q_matrix @ (1/2 * np.block([[np.zeros(3)], [np.eye(3)]]))
        
        Xdx = np.block([
            [  Q_delta_theta, np.zeros((4,n))],
            [np.zeros((n,3)),       np.eye(n)]
        ])
        
        return Xdx