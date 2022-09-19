import numpy as np
from math import floor



class MotionPlanner:
    # Constructor
    def __init__(self):

        ### WBC parameters

        # Robot torques limits (they are the same for all the joints)
        self.tau_max =   80
        self.tau_min = - 80

        # Normal contact force limits
        self.Fn_max = 350
        self.Fn_min =  40
        
        # Friction coefficient
        self.mu = 0.8

        # Controller time step
        self.dt = 1 / 200


        ### Locomotion parameters
        
        # Gait pattern sequence
        self.gait_pattern = ['LF', 'RH', 'RF', 'LH']

        # Period between two consecutive footfalls of the same foot (eg: LF)
        self.cycle_duration = 4

        # Ratio of the time used to move the com during a single step and the time for all the step. (During the step there are two phases: first the center of mass is moved and then the foot is moved).
        self.t_com_t_leg_cycle = 0.7

        # Lenght of a single step
        self.step_length = 0.14

        # Maximum height of a step
        self.step_height = 0.1

        # Spline order for the generation of the swing feet trajectoriesko
        self.spline_order = 3

        # Desired base height
        self.h_b_des = 0.47
        
        # Initial height
        self.h_init = 0.6

        # Time normalized stride phase
        self.phi = 0

        # Number of total gait cycles completed
        self.cycles_completed = 0

        # Absolute position of the feet in the x and y coordinate (the legs are symmetric with respect to the base position, it seems).
        self.abs_leg_pos = np.array([0.4, 0.3])


    # ========================== Get_wbc_parameters ========================== #

    ### Get some parameters used in the whole body controller

    def get_wbc_parameters(self, nc):

        # Robot torques limits (they are the same for all the joints)
        tau_max = self.tau_max
        tau_min = self.tau_min

        # Normal contact forces limits
        # Passing them in this way (one force limit per limb in contact with the terrain) is done to later enable the planner to express the normal force limits as a function of the time normalized stride phase
        Fn_max = self.Fn_max * np.ones(nc)
        Fn_min = self.Fn_min * np.ones(nc)

        # Friction coefficient
        mu = self.mu

        # Sample time of the controller
        dt = self.dt

        return tau_max, tau_min, Fn_max, Fn_min, mu, dt


    # ======================== Motion_planner_template ======================= #

    # Template for the real locomotion planner. It takes as input the number of contact feet and returns the generalized desired pose: the set of vectors that defines the desired position, velocity, and acceleration of the base, the desired angular acceleration and orientation of the base, and the desired position, velocity, and acceleration of the swing feet (this last part has a variable size depending on the number of feet currently in swing phase).
    # Using this function will result in a robot that is simply standing still.

    def motion_planner_template(self, nc):

        # Number of swing feet
        ns = 4 - nc

        # Base position quantities
        r_b_ddot_des = np.zeros(3)
        r_b_dot_des = np.zeros(3)
        r_b_des = np.array([0.0,0.0,0.5])

        # Base angular quantities
        omega_des = np.zeros(3)
        q_des = np.array([0.,0.,0.,1.])
        

        # Swing feet position quantities
        r_s_ddot_des = np.tile(np.array([0,0,-100]), ns)
        r_s_dot_des = np.zeros(3*ns)
        r_s_des = np.zeros(3*ns)

        return r_b_ddot_des, r_b_dot_des, r_b_des, omega_des, q_des, r_s_ddot_des, r_s_dot_des, r_s_des


    # ============================== Raise_foot ============================== #

    ### Planner for the raise foot task

    def raise_foot(self):

        if self.phi < 0.6:
            # Initialization phase in which all the feet are in contact with the terrain
            contactFeet = ['LF','RF','LH','RH']
            r_s_des = np.empty(0)
        else:
            # Raise the left front foot
            r_s_des = np.array([self.abs_leg_pos[0],self.abs_leg_pos[1],0.15])
            contactFeet = ['RF','LH','RH']


        # Compute the other quantities that define the desired generalized pose

        nc = len(contactFeet)
        # Number of swing feet
        ns = 4 - nc

        DeltaT = 0.4

        base_pos = np.array([-0.05, -0.05, 0.55])

        if self.phi < 0.15:

            # Base position quantities
            r_b_ddot_des = np.zeros(3)
            r_b_dot_des = np.zeros(3)
            r_b_des = np.array([0.0,0.0,0.55])
            
        elif self.phi > 0.15 + DeltaT:

            # Base position quantities
            r_b_ddot_des = np.zeros(3)
            r_b_dot_des = np.zeros(3)
            r_b_des = base_pos

        elif (self.phi > 0.15 and self.phi < 0.15 + DeltaT):

            r_b_des, r_b_dot_des, r_b_ddot_des = self._spline(np.array([0.0,0.0,0.55]), base_pos, (self.phi - 0.15) / DeltaT)

        # Base angular quantities
        omega_des = np.zeros(3)
        q_des = np.array([0,0,0,1])
    
        # Swing feet position quantities
        r_s_ddot_des = np.tile(np.array([0,0,0]), ns)
        r_s_dot_des = np.zeros(3*ns)


        # Increase the phase
        self.phi += self.dt / self.cycle_duration


        return contactFeet, r_b_ddot_des, r_b_dot_des, r_b_des, omega_des, q_des, r_s_ddot_des, r_s_dot_des, r_s_des


    # ================================ _spline =============================== #

    # Spline from pi to pf with parameter t in [0,1]
    # With a fifth order spline it is possible to set both the velocity and the acceleration to zero at the start and at the end of the spline
    # With a third order spline it is possible to set only the velocity of the point to zero at the start and at the end of the spline (in addition to setting the initial and final spline points).

    def _spline(self, pi, pf, t):
        
        # Local time of transition phase and its derivatives
        if self.spline_order == 5:

            f_t = t**3 * (10 - 15 * t + 6 * t**2)

            f_t_dot = 30 * t**2 - 60 * t**3 + 30 * t**4

            f_t_ddot = 60 * t - 180 * t**2 + 120 * t**3

        elif self.spline_order == 3:

            f_t = t**2 * (3 - 2*t)

            f_t_dot = 6*t - 6*t**2

            f_t_ddot = 6 - 12*t


        # Polynomial spline and its derivatives
        p_t = (1 - f_t) * pi + f_t * pf

        v_t = - f_t_dot * pi + f_t_dot * pf

        a_t = - f_t_ddot * pi + f_t_ddot * pf


        return p_t, v_t, a_t


    # ============================== Static_walk ============================= #

    # Forward static walking method.

    def static_walk(self):

        # Initialization quantities
        init_phase = 0.2
        init_pos = np.array([0.0,0.0,self.h_b_des])

        # These quantities define how much the base oscillates.
        #   LF      |      RF
        #           |
        #         2 | 4
        #   --------|------
        #         3 | 1
        #           |
        #   LH      |      RH
        r_b_osc_x = 0.06
        r_b_osc_y = 0.06
        r_b_osc = np.array([
            [ - r_b_osc_x, - r_b_osc_y, init_pos[2]],       # place the base here before lifting the left front foot
            [   r_b_osc_x,   r_b_osc_y, init_pos[2]],       # place the base here before lifting the right hind foot
            [ - r_b_osc_x,   r_b_osc_y, init_pos[2]],       # place the base here before lifting the left hind foot
            [   r_b_osc_x, - r_b_osc_y, init_pos[2]]        # place the base here before lifting the right front foot
            ])

        # Position of the robot legs in the base frame
        abs_legs_pos = np.array([
            [ + self.abs_leg_pos[0], + self.abs_leg_pos[1]],    # left front foot
            [ - self.abs_leg_pos[0], - self.abs_leg_pos[1]],    # right hind foot
            [ + self.abs_leg_pos[0], - self.abs_leg_pos[1]],    # left hind foot
            [ - self.abs_leg_pos[0], + self.abs_leg_pos[1]]     # right front foot
        ])

        # List of all the feet abbreviated names
        allFeet = ['LF','RF','LH','RH']

        # Step vector
        Step = np.array([self.step_length, 0, 0])

        # Step height
        step_height = self.step_height

        #
        desired_foot_penetration = 0.01


        # Initialization phase, first part
        if self.phi < init_phase / 2:
            # Base linear position quantities
            r_b_ddot_des = np.zeros(3)
            r_b_dot_des = np.zeros(3)
            r_b_des = np.array([0.0,0.0,self.h_init]) + self.phi / (init_phase / 2) * (init_pos - np.array([0.0,0.0,self.h_init]))

            # Contact Feet list
            contactFeet = allFeet

            # Swing feet quantities
            r_s_des = np.empty(0)
            r_s_dot_des = np.empty(0)
            r_s_ddot_des = np.empty(0)

        # Initialization phase, second part
        elif self.phi < init_phase:
            # Base linear position quantities
            init_end_pos = r_b_osc[3]
            t_phi = (self.phi - init_phase/2) / (init_phase/2)
            r_b_des, r_b_dot_des, r_b_ddot_des = self._spline(init_pos, init_end_pos, t_phi)

            # Contact Feet list
            contactFeet = allFeet

            # Swing feet quantities
            r_s_des = np.empty(0)
            r_s_dot_des = np.empty(0)
            r_s_ddot_des = np.empty(0)

        # If the initialization phase has ended, run this code
        else:
            # Consider a new phase independent on the initialization time
            phi = self.phi - init_phase

            swing_foot_id = floor(4 * phi)

            # Update the position of the base and of the legs to take into account how much the robot has walked.
            r_b_osc += Step/4 * swing_foot_id + Step * self.cycles_completed

            abs_legs_pos[swing_foot_id] += Step[0:2] * self.cycles_completed

            # In the first part of the motion the base is moved from the old position to the new position (on the opposite side of the foot that will be raised) in order to guarantee static stability.
            if 4 * phi - swing_foot_id < self.t_com_t_leg_cycle:
                phi_2 = 4 * phi - swing_foot_id

                DeltaT = self.t_com_t_leg_cycle

                r_b_des, r_b_dot_des, r_b_ddot_des = self._spline(r_b_osc[swing_foot_id - 1], r_b_osc[swing_foot_id], phi_2 / DeltaT)

                contactFeet = allFeet
                r_s_des = np.empty(0)
                r_s_dot_des = np.empty(0)
                r_s_ddot_des = np.empty(0)

            # After the base has been moved to the new position that guarantees static stability, move the foot.
            else:
                # The desired base position, velocity, and acceleration are computed by calculating the initial and final position of the base, and by allocating the appropriate time requirements for this movement.
                # The base movement from one r_b_osc to the successive should happen in 1/4 * self.t_com_t_leg_cycle.

                phi_2 = 4 * phi - swing_foot_id - self.t_com_t_leg_cycle

                DeltaT = (1 - self.t_com_t_leg_cycle)

                # Compute the base desired quantities by using a polynomial spline. Such a spline is used in order to have null initial and final velocity and acceleration.
                r_b_des, r_b_dot_des, r_b_ddot_des = self._spline(r_b_osc[swing_foot_id], r_b_osc[swing_foot_id] + Step / 4, phi_2 / DeltaT)

                # Obtain the list of feet in contact with the terrain by removing the swing_foot_id foot. This assures that the list has the correct order and is consistent with the other parts of the code.
                contactFeet = allFeet
                contactFeet.remove(self.gait_pattern[swing_foot_id])


                # The desired position, velocity, and acceleration of the swing feet are obtained by considering two motions: an horizontal motion from the initial foot position to the final foot position (equal to init_foot_pos + step_vector) and a vertical motion that first raises and than lowers the foot.

                # Leg initial and final horizontal position
                leg_init_pos = np.concatenate((abs_legs_pos[swing_foot_id], np.array([0.0])))
                leg_end_pos = np.concatenate((abs_legs_pos[swing_foot_id] + np.array([self.step_length, 0]), np.array([0.0])))

                # Horizontal part of the desired position, velocity, and acceleration of the swing feet
                r_s_des, r_s_dot_des, r_s_ddot_des = self._spline(leg_init_pos, leg_end_pos, phi_2 / DeltaT)

                # This is used to compute the vertical component of the foot position, velocity and acceleration while the foot is being raised.
                if 4 * phi - swing_foot_id < (1 - self.t_com_t_leg_cycle) / 2 + self.t_com_t_leg_cycle:
                    phi_2 = 4 * phi - swing_foot_id - self.t_com_t_leg_cycle

                    DeltaT = (1 - self.t_com_t_leg_cycle) / 2

                    leg_init_pos = np.concatenate((abs_legs_pos[swing_foot_id], np.array([0.0])))
                    leg_end_pos = np.concatenate((abs_legs_pos[swing_foot_id], np.array([step_height])))

                    r_s_des_2, r_s_dot_des_2, r_s_ddot_des_2 = self._spline(leg_init_pos, leg_end_pos, phi_2 / DeltaT)

                # This part is used to compute the vertical component of the foot position, velocity and acceleration while the foot is being lowered.
                else:
                    phi_2 = 4 * phi - swing_foot_id - self.t_com_t_leg_cycle - (1 - self.t_com_t_leg_cycle) / 2

                    DeltaT = (1 - self.t_com_t_leg_cycle) / 2

                    leg_init_pos = np.concatenate((abs_legs_pos[swing_foot_id], np.array([step_height])))
                    leg_end_pos = np.concatenate((abs_legs_pos[swing_foot_id], np.array([- desired_foot_penetration])))

                    r_s_des_2, r_s_dot_des_2, r_s_ddot_des_2 = self._spline(leg_init_pos, leg_end_pos, phi_2 / DeltaT)
                
                # Compute the total desired position, velocity, and acceleration of the swing foot by superimposing to the initial horizontal movement the vertical movement obtained in the previous if-else cycle.
                r_s_des[2] += r_s_des_2[2]
                r_s_dot_des[2] += r_s_dot_des_2[2]
                r_s_ddot_des[2] += r_s_ddot_des_2[2]



        # Base angular quantities
        omega_des = np.zeros(3)
        q_des = np.array([0,0,0,1])

        

        # Increase the phase
        self.phi += self.dt / self.cycle_duration

        # If the phase is greater than one, reset it to 0 (plus the init_phase).
        if self.phi - init_phase > 1:
            self.phi -= 1
            self.cycles_completed += 1


        return contactFeet, r_b_ddot_des, r_b_dot_des, r_b_des, omega_des, q_des, r_s_ddot_des, r_s_dot_des, r_s_des