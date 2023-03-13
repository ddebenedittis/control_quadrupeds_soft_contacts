import numpy as np
from scipy.linalg import block_diag
from quadprog import solve_qp

from .interpolation import Interpolation, InterpolationMethod



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
        self.theta0 = 0.61
        self.r = 0.5

        # Current desired yaw
        self.dtheta = 0
        
        self.interp = Interpolation()
        self.interp.method = InterpolationMethod.spline_5th
        
        # Swing phase duration
        self.interp.Ts = 0.2
        
        self.interp.horizontal_delay = 0.0


        # Current feet position
        self.p0_swing_feet = [
            np.array([  0.43,   0.3, 0.0]),     # LF
            np.array([  0.43, - 0.3, 0.0]),     # RF
            np.array([- 0.43,   0.3, 0.0]),     # LH
            np.array([- 0.43, - 0.3, 0.0]),     # RH         
        ]


        # Desired center of mass height
        self.zcom = 0.5
        
    
    # ================================== Ts ================================== #
    
    def get_Ts(self):
        return self.interp.Ts
    
    def set_Ts(self, value):
        self.interp.Ts = value
        
    Ts = property(get_Ts, set_Ts)


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

    def mpc(self, p_com, v_com, a_com, vel_cmd, yaw_rate_cmd):
        """
        Main method called to perform a step of the planner.
        """

        # Initial COM position.
        Xcom_0 = np.array([p_com[0], v_com[0]])
        Ycom_0 = np.array([p_com[1], v_com[1]])

        # Initial ZMP position (from the dynamics of a linear inverted pendulum).
        g = 9.81
        px_0 = p_com[0] - a_com[0] * self.zcom / g
        py_0 = p_com[1] - a_com[1] * self.zcom / g

        # Compute the x and y coordinates of the ZMP.
        xcom_dot_des = vel_cmd[0]; ycom_dot_des = vel_cmd[1]
        p_x_star = self._mpc_qp(Xcom_0, px_0, xcom_dot_des)
        p_y_star = self._mpc_qp(Ycom_0, py_0, ycom_dot_des)

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

        return contactFeet, r_b_ddot_des, r_b_dot_des, r_b_des, omega_des, q_des, r_s_ddot_des, r_s_dot_des, r_s_des