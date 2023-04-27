from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import block_diag
from quadprog import solve_qp

from .interpolation import Interpolation, InterpolationMethod



# ============================================================================ #
#                      DESIRED GENERALIZED POSE DATACLASS                      #
# ============================================================================ #

@dataclass
class DesiredGeneralizedPose():
    base_acc: np.ndarray = np.zeros(3)
    base_vel: np.ndarray = np.zeros(3)
    base_pos: np.ndarray = np.zeros(3)
    
    base_angvel: np.ndarray = np.zeros(3)
    base_quat: np.ndarray = np.array([1., 0., 0., 0.])
    
    feet_acc: np.ndarray = np.empty([0])
    feet_vel: np.ndarray = np.empty([0])
    feet_pos: np.ndarray = np.empty([0])
    contact_feet_names: list[str] = field(default_factory=list) 



# ============================================================================ #
#                                 MOTIONPLANNER                                #
# ============================================================================ #

# Class used to compute the desired generalized pose with a trotting gait.

class MotionPlanner:

    # Class constructor
    def __init__(self):

        # Number of future steps over which the optimization is performed
        self.N = 3

        # Cost function matrices
        # cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]
        self.Q = 1000 * np.eye(self.N)
        self.R = 1 * np.eye(self.N)

        # Planner time step
        self.dt = 1 / 200

        # Time normalized stride phase
        self.phi = 0


        # Current swing feet
        self.swing_feet = ['LF', 'RH']
        
        # Kinematic reachability limit for the stance feet
        self.d = 0.2

        # Maximum height of a step
        self.step_height = 0.1

        # Parameters used to describe the feet position with respect to the ZMP
        self.theta0 = 0.64
        self.r = 0.5

        # Current desired yaw
        self.dtheta = 0
        
        self.interp = Interpolation()
        self.interp.method = InterpolationMethod.spline_5th
        
        # Swing phase duration
        self.interp.Ts = 0.2
        
        self.interp.horizontal_delay = 0.0


        # Current feet position
        self.p0_swing_feet = []
        
        self.p_swing_feet = []
        
        self.update_initial_conditions()


        # Desired center of mass height
        self.zcom = 0.5
        
        
        # The planner stops moving the feet when the reference velocity (linear and angular) has been equal to zero for a number of steps equal to max_fixed_steps.
        self.max_fixed_steps = 6
        self.fixed_steps = 6
        
    
    # ================================== Ts ================================== #
    
    def get_Ts(self):
        return self.interp.Ts
    
    def set_Ts(self, value):
        self.interp.Ts = value
        
    Ts = property(get_Ts, set_Ts)
    
    
    # ======================= Update_initial_conditions ====================== #
    
    def update_initial_conditions(self, p_0 = np.zeros(3), yaw = 0):
        self.dtheta = yaw
        
        self.p0_swing_feet = [
            np.array([  self.r * np.cos(self.theta0 + self.dtheta),   self.r * np.sin(self.theta0 + self.dtheta), 0.0]),     # LF
            np.array([  self.r * np.cos(self.theta0 - self.dtheta), - self.r * np.sin(self.theta0 - self.dtheta), 0.0]),     # RF
            np.array([- self.r * np.cos(self.theta0 - self.dtheta),   self.r * np.sin(self.theta0 - self.dtheta), 0.0]),     # LH
            np.array([- self.r * np.cos(self.theta0 + self.dtheta), - self.r * np.sin(self.theta0 + self.dtheta), 0.0]),     # RH         
        ]
        self.p0_swing_feet += p_0
        
        self.p_swing_feet = self.p0_swing_feet


    # ======================================================================== #

    # Inverted pendulum linearized dynamics

    # Xcom_i = [xcom_i; xcom_dot_i]
    # Xcom_i = A(Ts) Xcom_i-1 + b(Ts) p_i

    @staticmethod
    def A_t(omega, t):
        A = np.block([
            [np.cosh(omega*t),         omega**-1 * np.sinh(omega*t)],
            [omega * np.sinh(omega*t), np.cosh(omega*t)]
        ])

        return A


    @staticmethod
    def b_t(omega, t):
        b = np.block([
            1 - np.cosh(omega*t),
            - omega * np.sinh(omega*t)
        ])

        return b


    # ================================ _mpc_qp =============================== #

    # Model predictive control based on QP

    def _mpc_qp(self, Xcom_0, p_0, xcom_dot_des):

        ### State
        
        # xi = [
        #     1                             extra state equal to one
        #     [xcom_i, x_com_dot_i].T       position and velocity coordinate (only one coordinate)
        #     px                            ZMP coordinate
        # ]

        N = self.N      # number of future steps
        Ts = self.Ts    # swing phase duration


        ### QP formulation

        # min_xi   1/2 xi^T H xi + c^T xi
        # s.t.: A xi - b  = 0
        #       D xi - f <= 0


        ### Cost definition

        # cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]

        # Part of the cost associated with the com velocity error
        H11 = np.kron(self.Q, block_diag(0,1))
        # Part of the cost associated with the zmp displacement with respect to the previous step
        H22 = self.R + block_diag(np.diag(np.diag(self.R[1:,1:])), 0) - np.diag(np.diag(self.R[1:,1:]), k=1) - np.diag(np.diag(self.R[1:,1:]), k=-1)

        H = block_diag(0, H11, H22)


        xcom_dot_des = xcom_dot_des * np.ones(N)

        c0 = np.array([xcom_dot_des.T @ self.Q @ xcom_dot_des + self.R[0,0] * p_0**2])

        c1 = np.kron(- 2 * self.Q @ xcom_dot_des, np.array([0,1]))

        c2 = np.block([
            - 2 * self.R[0,0] * p_0,
            np.zeros(N-1)
        ])

        c = np.concatenate([c0, c1, c2])


        ### Equality constraints
        # Dynamic consistency with the linearized inverted pendulum model

        g = 9.81
        omega = (g / self.zcom)**0.5

        A_t = MotionPlanner.A_t(omega, Ts)
        b_t = MotionPlanner.b_t(omega, Ts)
        
        A = block_diag(
            1,
            np.block([- np.eye(2*N) + np.kron(np.eye(N, k=-1), A_t), np.kron(np.eye(N), np.array([[b_t[0]],[b_t[1]]]))])
        )

        t0 = self.Ts * (1 - self.phi)
        A_t0 = MotionPlanner.A_t(omega, t0)
        b_t0 = MotionPlanner.b_t(omega, t0)

        b = np.block([
            1,
            - A_t @ (A_t0 @ Xcom_0 + b_t0 * p_0),
            np.zeros(2*(N-1))
        ])


        ### Inequality constraints
        # Limit the zmp displacement between two consecutive steps

        D11 = np.eye(N) - np.eye(N, k=-1)
        D11 = np.block([np.zeros([N,1]), np.zeros([N,2*N]), D11])

        D = np.vstack((D11, -D11))

        f = np.block([
            p_0 + self.d,
            np.ones(N-1) * self.d,
            - p_0 + self.d,
            np.ones(N-1) * self.d
        ])


        ### quadprog QP formulation

        # min  1/2 x^T G x - a^T x
        # s.t. C^T x >= b
        # where the first meq are equality constraints and the remaining ones are inequality constraints

        reg = 1e-9
        H = H + reg * np.eye(3*N+1)

        sol = solve_qp(H, -c, - np.vstack((A, D)).T, - np.concatenate((b, f)), 2*N+1)[0]

        p_i_star = sol[1+2*self.N]

        return p_i_star


    # ========================== _desired_footholds ========================== #

    # Compute the desired footholds for all the feet currently in swing phase

    def _desired_footholds(self, p_star):

        # Initialize the output footholds positions
        p_swing_feet = []

        r = self.r

        dtheta = self.dtheta

        for i in self.swing_feet:

            if   i == 'LF':
                p_swing_feet.append(p_star + r * np.array([  np.cos(self.theta0 + dtheta),   np.sin(self.theta0 + dtheta)]))

            elif i == 'RF':
                p_swing_feet.append(p_star + r * np.array([  np.cos(self.theta0 - dtheta), - np.sin(self.theta0 - dtheta)]))

            elif i == 'LH':
                p_swing_feet.append(p_star + r * np.array([- np.cos(self.theta0 - dtheta),   np.sin(self.theta0 - dtheta)]))

            elif i == 'RH':
                p_swing_feet.append(p_star + r * np.array([- np.cos(self.theta0 + dtheta), - np.sin(self.theta0 + dtheta)]))
                
        self.p_swing_feet = p_swing_feet

        return p_swing_feet


    # =========================== _foot_trajectory =========================== #

    # Generate an appropriate spline for the feet 

    def _foot_trajectory(self, p_swing_feet):

        phi = self.phi
        
        all_feet = ['LF', 'RF', 'LH', 'RH']

        r_s_des = np.empty([0,])
        r_s_dot_des = np.empty([0,])
        r_s_ddot_des = np.empty([0,])

        for i in range(4):

            if all_feet[i] in self.swing_feet:
                
                p0 = self.p0_swing_feet[i]
                pf = p_swing_feet[ self.swing_feet.index(all_feet[i]) ]
                pf = np.concatenate((pf, np.array([0.0])))

                r_s_des_temp, r_s_dot_des_temp, r_s_ddot_des_temp = self.interp.interpolate(p0, pf, phi)
                
                r_s_des = np.hstack((r_s_des, r_s_des_temp))
                r_s_dot_des = np.hstack((r_s_dot_des, r_s_dot_des_temp))
                r_s_ddot_des = np.hstack((r_s_ddot_des, r_s_ddot_des_temp))

        return r_s_ddot_des, r_s_dot_des, r_s_des

    
    # ========================== _switch_swing_feet ========================== #

    # Change the list of feet in swing phase and update the position of the feet in contact with the terrain. (This position is used in order to create the spline once, in the next step, those feet will be in swing phase)

    def _switch_swing_feet(self, p_swing_feet):

        if self.swing_feet == ['LF', 'RH']:
            self.swing_feet = ['RF', 'LH']

            self.p0_swing_feet[0] = np.concatenate((p_swing_feet[0], np.array([0])))
            self.p0_swing_feet[3] = np.concatenate((p_swing_feet[1], np.array([0])))
        else:
            self.swing_feet = ['LF', 'RH']

            self.p0_swing_feet[1] = np.concatenate((p_swing_feet[0], np.array([0])))
            self.p0_swing_feet[2] = np.concatenate((p_swing_feet[1], np.array([0])))
            
            
    # ========================== _get_des_base_pose ========================== #

    # Compute the desired base position, velocity and acceleration from the current base position and ZMP position, using the LIPM (Linearized Inverted Pendulum Model). 

    def _get_des_base_pose(self, Xcom_0, Ycom_0, px_0, py_0):

        g = 9.81
        omega = (g / self.zcom)**0.5

        dt = self.dt
        
        Xcom = MotionPlanner.A_t(omega, dt) @ Xcom_0 + MotionPlanner.b_t(omega, dt) * px_0
        Ycom = MotionPlanner.A_t(omega, dt) @ Ycom_0 + MotionPlanner.b_t(omega, dt) * py_0

        r_b_des = np.array([Xcom[0], Ycom[0], self.zcom])
        r_b_dot_des = np.array([Xcom[1], Ycom[1], 0])
        r_b_ddot_des = g / self.zcom * np.array([Xcom_0[0] - px_0, Ycom_0[0] - py_0, 0])

        return r_b_ddot_des, r_b_dot_des, r_b_des
    
    
    # ================================== Mpc ================================= #
    
    def _mpc(self, p_com, v_com, a_com, vel_cmd, yaw_rate_cmd) -> DesiredGeneralizedPose:
        
        # Initial COM position.
        Xcom_0 = np.array([p_com[0], v_com[0]])
        Ycom_0 = np.array([p_com[1], v_com[1]])

        # Initial ZMP position (from the dynamics of a linear inverted pendulum).
        g = 9.81
        px_0 = p_com[0] - a_com[0] * self.zcom / g
        py_0 = p_com[1] - a_com[1] * self.zcom / g
        
        ph_0 =   px_0 * np.cos(self.dtheta) + py_0 * np.sin(self.dtheta)
        pl_0 = - px_0 * np.sin(self.dtheta) + py_0 * np.cos(self.dtheta)
        
        Hcom_0 =   Xcom_0 * np.cos(self.dtheta) + Ycom_0 * np.sin(self.dtheta)
        Lcom_0 = - Xcom_0 * np.sin(self.dtheta) + Ycom_0 * np.cos(self.dtheta)

        # Compute the x and y coordinates of the ZMP.
        xcom_dot_des = vel_cmd[0]; ycom_dot_des = vel_cmd[1]
        p_h_star = self._mpc_qp(Hcom_0, ph_0, xcom_dot_des)
        p_l_star = self._mpc_qp(Lcom_0, pl_0, ycom_dot_des)
        
        p_x_star = p_h_star * np.cos(self.dtheta) - p_l_star * np.sin(self.dtheta)
        p_y_star = p_h_star * np.sin(self.dtheta) + p_l_star * np.cos(self.dtheta)

        # Optimal ZMP position.
        p_star = np.array([p_x_star, p_y_star])

        # Compute the desired footholds of the feet currently in swing phase.
        self.dtheta += yaw_rate_cmd * self.dt
        p_swing_feet = self._desired_footholds(p_star)

        # Compute the feet trajectories.
        r_s_ddot_des, r_s_dot_des, r_s_des = self._foot_trajectory(p_swing_feet)

        # Compute the base linear trajectory.
        r_b_ddot_des, r_b_dot_des, r_b_des = self._get_des_base_pose(Xcom_0, Ycom_0, px_0, py_0)

        # Compute the base angular trajectory.
        omega_des = np.array([0.0, 0.0, yaw_rate_cmd])
        q_des = np.array([0.0, 0.0, np.sin(self.dtheta/2), np.cos(self.dtheta/2)])

        # Compute the list of feet in contact phase.
        contactFeet = []
        allFeet = ['LF', 'RF', 'LH', 'RH']
        for i in allFeet:
            if i not in self.swing_feet:
                contactFeet.append(i)

        # Update the normalized phase. When it becomes >=1, switch the contact feet and reset it to zero.
        self.phi += self.dt / self.Ts
        if self.phi >= 1:
            self.phi = 0
            self._switch_swing_feet(p_swing_feet)
            
            
        des_gen_pose = DesiredGeneralizedPose(
            base_acc=r_b_ddot_des,
            base_vel=r_b_dot_des,
            base_pos=r_b_des,
            base_angvel=omega_des,
            base_quat=q_des,
            feet_acc=r_s_ddot_des,
            feet_vel=r_s_dot_des,
            feet_pos=r_s_des,
            contact_feet_names=contactFeet
        )
        
        return des_gen_pose
    
    
    # ============================== _check_stop ============================= #
    
    def _check_stop(self, p_com, vel_cmd, yaw_rate_cmd) -> tuple[bool, DesiredGeneralizedPose]:
        """
        Check wheter the robot should stop. If so, keep the desired generalized pose fixed.

        Args:
            p_com (np.ndarray): center of mass position
            vel_cmd (np.ndarray): commanded linear velocity
            yaw_rate_cmd (float): commanded yaw rate

        Returns:
            tuple[bool, DesiredGeneralizedPose]: flag that is true when the planner has been stopped and the desired generalized pose
        """
        
        # Stop moving the feet if the commanded velocity has been equal to zero for a number of steps equal to max_fixed_steps.
        
        if self.phi == 0 and self.fixed_steps >= self.max_fixed_steps \
            and np.linalg.norm(vel_cmd) < 0.01 and yaw_rate_cmd < 0.01:
                
            # Stop the robot movement.
                
            r_b_des = np.array([p_com[0], p_com[1], self.zcom])
            q_des = np.array([0.0, 0.0, np.sin(self.dtheta/2), np.cos(self.dtheta/2)])
            
            des_gen_pose = DesiredGeneralizedPose(
                base_acc=np.zeros(3),
                base_vel=np.zeros(3),
                base_pos=r_b_des,
                base_angvel=np.zeros(3),
                base_quat=q_des,
                feet_acc=np.zeros(0),
                feet_vel=np.zeros(0),
                feet_pos=np.zeros(0),
                contact_feet_names=['LF', 'RF', 'LH', 'RH']
            )
            
            return True, des_gen_pose
        elif self.phi == 0 and self.fixed_steps < self.max_fixed_steps \
            and np.linalg.norm(vel_cmd) < 0.01 and yaw_rate_cmd < 0.01:
            
            # Do not stop the robot, but increase the number of steps during which the commanded twist was zero.
            self.fixed_steps += 1
        elif np.linalg.norm(vel_cmd) > 0.01 or yaw_rate_cmd > 0.01:
            # The commanded twist is not null. Reset the fixed_steps variable
            self.fixed_steps = 0
            
        return False, DesiredGeneralizedPose()


    # ================================ Update ================================ #

    def update(self, p_com, v_com, a_com, vel_cmd, yaw_rate_cmd) -> DesiredGeneralizedPose:
        """
        Main method called to perform an update of the planner.

        Args:
            p_com (np.ndarray): position of the center of mass
            v_com (np.ndarray): velocity of the center of mass
            a_com (np.ndarray): acceleration of the center of mass
            vel_cmd (np.ndarray): linear velocity command
            yaw_rate_cmd (float): yaw rate command

        Returns:
            DesiredGeneralizedPose: desired generalized pose computed by the trotting planner
        """
        
        # Stop moving the feet if the commanded velocity has been equal to zero for a number of steps equal to max_fixed_steps.
        stop_flag, des_gen_pose = self._check_stop(p_com, vel_cmd, yaw_rate_cmd)
        if stop_flag:
            return des_gen_pose
        
        # The robot should not stop, compute the mpc for trotting.
        des_gen_pose = self._mpc(p_com, v_com, a_com, vel_cmd, yaw_rate_cmd)

        return des_gen_pose
    
    
    # ======================= Trajectory_sample_points ======================= #
    
    def trajectory_sample_points(self):
        """
        Return the list containing the trajectories of the swing feet.
        """
        
        all_feet = ['LF', 'RF', 'LH', 'RH']
        
        # The swing trajectory is sampled in n_points number of points.
        n_points = 10
        
        # It has n_trajectories * 3 columns.
        r_s_des = [np.zeros((n_points, 3)), np.zeros((n_points, 3))]
        
        feet_ids = []
        
        for i in range(4):
            if all_feet[i] in self.swing_feet:
                feet_ids.append(i)
        
        
        for i in range(n_points):
            
            phi = i / (n_points - 1)

            for j in feet_ids:
                    
                p0 = self.p0_swing_feet[j]
                pf = self.p_swing_feet[ self.swing_feet.index(all_feet[j]) ]
                pf = np.concatenate((pf, np.array([0.0])))

                r_s_des_temp = self.interp.interpolate(p0, pf, phi)[0]
                                
                jj = feet_ids.index(j)
                r_s_des[jj][i, :] = r_s_des_temp

        return r_s_des
