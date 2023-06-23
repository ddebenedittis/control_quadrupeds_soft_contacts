import csv
import datetime
from enum import Enum
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ContactsState
from generalized_pose_msgs.msg import GeneralizedPosesWithTime
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from velocity_command_msgs.msg import SimpleVelocityCommand



# ============================ Quaternion Distance =========================== #

def q_dist(q1, q2):

    theta = np.arccos(2 * np.dot(q1, q2) - 1)

    return theta


# ================================= ParamName ================================ #

class ParamName(Enum):
    STEP_LENGTH = 0
    CYCLE_DURATION = 1


# ============================================================================ #
#                               LoggerSubscriber                               #
# ============================================================================ #

class LoggerSubscriber(Node):
    """

    """

    def __init__(self):
        super().__init__("logger_subscriber")

        # ============================ Subscribers =========================== #

        self.optimal_joints_accelerations_subscription = self.create_subscription(
            Float64MultiArray,
            "logging/optimal_joints_accelerations",
            self.optimal_joints_accelerations_callback,
            1)

        self.optimal_torques_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/optimal_torques",
            self.optimal_torques_callback,
            1)

        self.optimal_forces_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/optimal_forces",
            self.optimal_forces_callback,
            1)

        self.optimal_deformations_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/optimal_deformations",
            self.optimal_deformations_callback,
            1)


        self.feet_positions_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/feet_positions",
            self.feet_positions_callback,
            1)

        self.feet_velocities_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/feet_velocities",
            self.feet_velocities_callback,
            1)


        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)

        self.joint_states_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            1)

        self.gen_pose_subscription = self.create_subscription(
            GeneralizedPosesWithTime,
            "/motion_planner/desired_generalized_poses",
            self.gen_pose_callback,
            1)


        self.contact_sensor_LF_subscription = self.create_subscription(
            ContactsState,
            "/contact_force_sensors/LF",
            self.contact_state_LF_callback,
            1)
        self.contact_sensor_RF_subscription = self.create_subscription(
            ContactsState,
            "/contact_force_sensors/RF",
            self.contact_state_RF_callback,
            1)
        self.contact_sensor_LH_subscription = self.create_subscription(
            ContactsState,
            "/contact_force_sensors/LH",
            self.contact_state_LH_callback,
            1)
        self.contact_sensor_RH_subscription = self.create_subscription(
            ContactsState,
            "/contact_force_sensors/RH",
            self.contact_state_RH_callback,
            1)


        # ================ Variables Saved By The Subscribers ================ #

        # q = [q_b, q_j]
        # Base pose: q_b = [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
        # Joint angles: q_j
        self.q = np.zeros(19)
        self.q[6] = 1

        # v = [v_b, v_j]
        # Base velocity: v_b = [v_x, v_y, v_z, w_x, w_y, w_z]
        # Joint velocities: v_j
        self.v = np.zeros(18)

        # Output of the optimization problem solution
        self.optimal_joints_accelerations = np.zeros(18)
        self.optimal_torques = np.zeros(12)
        self.optimal_forces = np.zeros(12)
        self.optimal_deformations = np.zeros(12)

        # Computed and published by the whole-body controller
        self.feet_positions = np.zeros((3,4))
        self.feet_velocities = np.zeros(12)

        # Planner output
        self.desired_base_pos = np.zeros(3)
        self.desired_base_quat = np.zeros(4); self.desired_base_quat[3] = 1
        self.desired_base_vel = np.zeros(3)
        self.desired_feet_pos = np.nan * np.zeros(12)
        self.desired_feet_vel = np.nan * np.zeros(12)
        self.contact_feet_names = []

        # Contact sensor
        self.contact_forces = np.zeros(12)
        self.contact_positions = np.zeros(12)
        self.depths = np.zeros(4)


        # =============================== Timer ============================== #

        # The locomotion data is saved inside the timer_callback.

        timer_period = 1/100
        self.timer = self.create_timer(timer_period, self.timer_callback)


        # ============================= Slippage ============================= #

        # Generic names of the feet in contact with the ground
        self.generic_feet_names = ['LF', 'RF', 'LH', 'RH']

        # Matrix containing the positions of the feet in contact with the terrain. The feet are supposed to be in contact with the terrain when the planner commands them to be in contact with the terrain.
        # TODO: use contact forces informations to compute this quantity.
        self.saved_feet_pos = np.full((3,4), np.nan)

        # Threshold before which the slippage is not computed
        self.threshold = 0.001


        # ==================== Read Other Nodes Parameters =================== #

        self.declare_parameter("robot_name", "anymal_c")
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        self.declare_parameter("gait", "static_walk")
        self.gait = self.get_parameter("gait").get_parameter_value().string_value

        if self.gait == "static_walk":
            self.step_length = 0
            self.cycle_duration = 0

            # Step length
            self.client_step_length = self.create_client(
                GetParameters, "/planner/get_parameters")

            request_step_length = GetParameters.Request()
            request_step_length.names = ["step_length"]

            self.client_step_length.wait_for_service()

            future_step_length = self.client_step_length.call_async(request_step_length)
            future_step_length.add_done_callback(self.callback_param_step_length)

            # Cycle duration
            self.client_cycle_duration = self.create_client(
                GetParameters, "/planner/get_parameters")

            request_cycle_duration = GetParameters.Request()
            request_cycle_duration.names = ["cycle_duration"]

            self.client_cycle_duration.wait_for_service()

            future_cycle_duration = self.client_cycle_duration.call_async(request_cycle_duration)
            future_cycle_duration.add_done_callback(self.callback_param_cycle_duration)
        elif self.gait == "walking_trot" or self.gait == "teleop_walking_trot":
            self.vel_forward_cmd = 0.
            self.vel_lateral_cmd = 0.
            self.yaw_rate_cmd = 0.

            self.teleop_vel_cmd_subscription = self.create_subscription(
                SimpleVelocityCommand,
                "/motion_generator/simple_velocity_command",
                self.simple_velocity_command_callback,
                1)


        # ============================= Csv Data ============================= #

        # Approximate time over which data is saved. Approximate because it depends on the actual frequency of the timer_callback.
        time_horizon = 10

        # Number of datapoints
        self.n = int(time_horizon / timer_period + 1)
        # Current step
        self.i = 0

        # Vectors that save the locomotion data. These vectors, once filled, are saved as CSVs.

        self.time_vector = np.zeros(self.n)

        self.gen_coordinates_vector = np.zeros((self.n, 19))
        self.gen_velocities_vector = np.zeros((self.n, 18))

        self.optimal_joints_accelerations_vector = np.zeros((self.n, 18))
        self.optimal_torques_vector = np.zeros((self.n, 12))
        self.optimal_forces_vector = np.zeros((self.n, 12))
        self.optimal_deformations_vector = np.zeros((self.n, 12))
        self.feet_positions_vector = np.zeros((self.n, 12))

        self.position_error_vector = np.zeros((self.n, 3))
        self.orientation_error_vector = np.zeros(self.n)
        self.velocity_error_vector = np.zeros((self.n, 3))

        self.desired_feet_positions_vector = np.zeros((self.n, 12))
        self.desired_feet_velocities_vector = np.zeros((self.n, 12))

        self.slippage_vector = np.zeros(self.n)

        self.contact_forces_vector = np.zeros((self.n, 12))
        self.contact_positions_vector = np.zeros((self.n, 12))
        self.depths_vector = np.zeros((self.n, 4))

        self.feet_positions_vector = np.zeros((self.n, 12))
        self.feet_velocities_vector = np.zeros((self.n, 12))

        if self.gait == "walking_trot" or self.gait == "teleop_walking_trot":
            self.simple_velocity_command_vector = np.zeros((self.n, 3))


    # =============================== Callbacks ============================== #

    def optimal_joints_accelerations_callback(self, msg):
        self.optimal_joints_accelerations = np.array(msg.data)

    def optimal_torques_callback(self, msg):
        self.optimal_torques = np.array(msg.data)

    def optimal_forces_callback(self, msg):
        self.optimal_forces = np.array(msg.data)

    def optimal_deformations_callback(self, msg):
        self.optimal_deformations = np.array(msg.data)

    def feet_positions_callback(self, msg):
        self.feet_positions = np.array(msg.data).reshape((4,3)).T

    def feet_velocities_callback(self, msg):
        self.feet_velocities = np.array(msg.data)


    def link_states_callback(self, msg):
        """
        Save the pose and twist of the robot base.
        """

        #! This assumes that there is only one link whose name ends with "base".
        for index in range(len(msg.name)):
            if msg.name[index][-4:] == "base":
                break

        # /gazebo/link_states returns the pose and the twist in the inertial or world frame.

        # Extract the base position and orientation (quaternion)
        pos = msg.pose[index].position
        orient = msg.pose[index].orientation

        # Extract the base linear and angular velocity
        lin = msg.twist[index].linear
        ang = msg.twist[index].angular

        # Save the base pose and twists as numpy arrays
        q_b = np.array([pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w])
        v_b = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])

        self.q[0:7] = q_b
        self.v[0:6] = v_b


    def joint_states_callback(self, msg: JointState):
        """
        Read, reorder and save the joint states.
        The joints are saved in the following order: LF_(HAA -> HFE -> KFE) -> LH -> RF -> RH,
        """

        if self.robot_name == "anymal_c" or self.robot_name == "anymal_c_softfoot_q":
            joint_names = ["LF_HAA", "LF_HFE", "LF_KFE",
                           "LH_HAA", "LH_HFE", "LH_KFE",
                           "RF_HAA", "RF_HFE", "RF_KFE",
                           "RH_HAA", "RH_HFE", "RH_KFE"]
        elif self.robot_name == "solo12":
            joint_names = ["FL_HAA", "FL_HFE", "FL_KFE",
                           "FR_HAA", "FR_HFE", "FR_KFE",
                           "HL_HAA", "HL_HFE", "HL_KFE",
                           "HR_HAA", "HR_HFE", "HR_KFE"]
        elif self.robot_name == "":
            return

        for i in range(len(joint_names)):
            self.q[7 + joint_names.index(msg.name[i])] = msg.position[i]
            self.v[6 + joint_names.index(msg.name[i])] = msg.velocity[i]


    def gen_pose_callback(self, msg: GeneralizedPosesWithTime):
        gen_pose = msg.generalized_poses_with_time[0].generalized_pose
        
        self.desired_base_pos = np.array([gen_pose.base_pos.x, gen_pose.base_pos.y, gen_pose.base_pos.z])
        self.desired_base_quat = np.array([gen_pose.base_quat.x, gen_pose.base_quat.y, gen_pose.base_quat.z, gen_pose.base_quat.w])
        self.desired_base_vel = np.array([gen_pose.base_vel.x, gen_pose.base_vel.y, gen_pose.base_vel.z])

        # These are generic feet names, independent on the particular quadrupedal robot. I.e. LF, RF, LH, RH.
        self.contact_feet_names = gen_pose.contact_feet

        j = 0

        for i, foot_name in enumerate(self.generic_feet_names):
            if foot_name not in self.contact_feet_names:
                self.desired_feet_pos[3*i:3*i+3] = gen_pose.feet_pos[3*j:3*j+3]
                self.desired_feet_vel[3*i:3*i+3] = gen_pose.feet_vel[3*j:3*j+3]
                j += 1
            else:
                self.desired_feet_pos[3*i:3*i+3] = np.nan * np.ones(3)
                self.desired_feet_vel[3*i:3*i+3] = np.nan * np.ones(3)


    def contact_state_callback(self, msg, foot_name):
        i = self.generic_feet_names.index(foot_name)

        # Compute the average of all the elements of the message
        self.contact_forces[0+3*i:3+3*i].fill(0)
        self.contact_positions[0+3*i:3+3*i].fill(0)
        self.depths[i] = 0

        # for contact_state in msg.states:
        #     self.contact_forces[0+3*i:3+3*i] += np.array([
        #         contact_state.total_wrench.force.x,
        #         contact_state.total_wrench.force.y,
        #         contact_state.total_wrench.force.z
        #     ])
        #     self.contact_positions[0+3*i:3+3*i] += np.array([
        #         contact_state.contact_positions[0].x,
        #         contact_state.contact_positions[0].y,
        #         contact_state.contact_positions[0].z
        #     ])
        #     self.depths[i] += contact_state.depths[0]

        # n = len(msg.states)

        # if n > 0:   # avoids dividing by zero
        #     self.contact_forces[0+3*i:3+3*i] = self.contact_forces[0+3*i:3+3*i] / n
        #     self.contact_positions[0+3*i:3+3*i] = self.contact_positions[0+3*i:3+3*i] / n
        #     self.depths[i] = self.depths[i] / n

        if len(msg.states) > 0:
            self.contact_forces[0+3*i:3+3*i] = np.array([
                msg.states[0].total_wrench.force.x,
                msg.states[0].total_wrench.force.y,
                msg.states[0].total_wrench.force.z
            ])

            self.contact_positions[0+3*i:3+3*i] = np.array([
                msg.states[0].contact_positions[0].x,
                msg.states[0].contact_positions[0].y,
                msg.states[0].contact_positions[0].z
            ])

            self.depths[i] = msg.states[0].depths[0]


    def contact_state_LF_callback(self, msg):
        self.contact_state_callback(msg, "LF")

    def contact_state_RF_callback(self, msg):
        self.contact_state_callback(msg, "RF")

    def contact_state_LH_callback(self, msg):
        self.contact_state_callback(msg, "LH")

    def contact_state_RH_callback(self, msg):
        self.contact_state_callback(msg, "RH")


    def simple_velocity_command_callback(self, msg):
        self.vel_forward_cmd = msg.velocity_forward
        self.vel_lateral_cmd = msg.velocity_lateral
        self.yaw_rate_cmd = msg.yaw_rate


    def compute_slippage(self, contact_feet_names, feet_pos):
        # Shortened and ordered feet names
        feet_names = self.generic_feet_names

        # Initialize the slippage at this time step
        slippage = 0

        for i in range(4):
            if feet_names[i] in contact_feet_names:
                if np.isnan(self.saved_feet_pos[0,i]):
                    # The foot was not in contact the previous time step. Update its saved position.
                    self.saved_feet_pos[:,i] = feet_pos[:,i]
                elif np.linalg.norm(self.saved_feet_pos[:,i] - feet_pos[:,i]) > self.threshold:
                    # The foot has slipped more than the threshold. Update the saved position and increase the slippage.
                    slippage += np.linalg.norm(self.saved_feet_pos[:,i] - feet_pos[:,i])
                    self.saved_feet_pos[:,i] = feet_pos[:,i]
            else:
                # When the foot is not in contact with the ground its position is set to nan.
                self.saved_feet_pos[:,i] = np.nan * np.ones(3)

        # If the slippage in the current timestep is too big, discard it.
        if slippage > 0.1:
            slippage = 0

        return slippage


    def callback_param(self, future, param_name: ParamName):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        else:
            if param_name == ParamName.STEP_LENGTH:
                self.step_length = result.values[0].double_value
            elif param_name == ParamName.CYCLE_DURATION:
                self.cycle_duration = result.values[0].double_value

    def callback_param_step_length(self, future):
        self.callback_param(future, ParamName.STEP_LENGTH)

    def callback_param_cycle_duration(self, future):
        self.callback_param(future, ParamName.CYCLE_DURATION)


    def timer_callback(self):
        """
        Fill the data if self.i < self.n. Then, save the data is some csv files and destroy the node.
        """

        # This optimal_deformations vector can have different sized depending on the contact model used in the controller. This chunk of code deals with this.
        if self.optimal_deformations.size != self.optimal_deformations_vector[0,:].size:
            self.optimal_deformations_vector = np.zeros((self.n, self.optimal_deformations.size))

        # =========================== Save The Data ========================== #

        self.i += 1

        self.time_vector[self.i] = self.get_clock().now().nanoseconds / 10**9

        self.gen_coordinates_vector[self.i, :] = self.q
        self.gen_velocities_vector[self.i, :] = self.v

        self.position_error_vector[self.i, :] = self.q[0:3] - self.desired_base_pos
        self.orientation_error_vector[self.i] = q_dist(self.q[3:7], self.desired_base_quat)
        self.velocity_error_vector[self.i, :] = self.v[0:3] - self.desired_base_vel

        self.desired_feet_positions_vector[self.i, :] = self.desired_feet_pos
        self.desired_feet_velocities_vector[self.i, :] = self.desired_feet_vel

        self.optimal_joints_accelerations_vector[self.i, :] = self.optimal_joints_accelerations
        self.optimal_torques_vector[self.i, :] = self.optimal_torques
        self.optimal_forces_vector[self.i, :] = self.optimal_forces
        self.optimal_deformations_vector[self.i, :] = self.optimal_deformations

        self.slippage_vector[self.i] = self.slippage_vector[self.i-1] + self.compute_slippage(self.contact_feet_names, self.feet_positions)

        self.contact_forces_vector[self.i, :] = self.contact_forces
        self.contact_positions_vector[self.i, :] = self.contact_positions
        self.depths_vector[self.i, :] = self.depths

        self.feet_positions_vector[self.i, :] = self.feet_positions.flatten('F')
        self.feet_velocities_vector[self.i, :] = self.feet_velocities

        if self.gait == "walking_trot" or self.gait == "teleop_walking_trot":
            self.simple_velocity_command_vector[self.i, :] = np.array([self.vel_forward_cmd, self.vel_lateral_cmd, self.yaw_rate_cmd])


        # ================= Save The Csv And Destroy The Node ================ #

        if self.i >= self.n - 1:
            path = "log/csv/" + f"{datetime.datetime.now():%Y-%m-%d-%H:%M:%S}" + "/"

            try:
                os.makedirs(path)
            except OSError:
                print("Creation of the directories %s failed" % path)
            else:
                print("Successfully created the directories %s" % path)

            np.savetxt(path+"time.csv", self.time_vector, delimiter=",")

            np.savetxt(path+"generalized_coordinates.csv", self.gen_coordinates_vector, delimiter=",")
            np.savetxt(path+"generalized_velocities.csv", self.gen_velocities_vector, delimiter=",")

            np.savetxt(path+"position_error.csv", self.position_error_vector, delimiter=",")
            np.savetxt(path+"orientation_error.csv", self.orientation_error_vector, delimiter=",")
            np.savetxt(path+"velocity_error.csv", self.velocity_error_vector, delimiter=",")

            np.savetxt(path+"desired_feet_positions.csv", self.desired_feet_positions_vector, delimiter=",")
            np.savetxt(path+"desired_feet_velocities.csv", self.desired_feet_velocities_vector, delimiter=",")

            np.savetxt(path+"optimal_joints_accelerations.csv", self.optimal_joints_accelerations_vector, delimiter=",")
            np.savetxt(path+"optimal_torques.csv", self.optimal_torques_vector, delimiter=",")
            np.savetxt(path+"optimal_forces.csv", self.optimal_forces_vector, delimiter=",")
            np.savetxt(path+"optimal_deformations.csv", self.optimal_deformations_vector, delimiter=",")

            np.savetxt(path+"slippage.csv", self.slippage_vector, delimiter=",")

            np.savetxt(path+"contact_forces.csv", self.contact_forces_vector, delimiter=",")
            np.savetxt(path+"contact_positions.csv", self.contact_positions_vector, delimiter=",")
            np.savetxt(path+"depths.csv", self.depths_vector, delimiter=",")

            np.savetxt(path+"feet_positions.csv", self.feet_positions_vector, delimiter=",")
            np.savetxt(path+"feet_velocities.csv", self.feet_velocities_vector, delimiter=",")

            if self.gait == "walking_trot" or self.gait == "teleop_walking_trot":
                np.savetxt(path+"simple_velocity_command.csv", self.simple_velocity_command_vector, delimiter=",")

            # ================================================================ #

            speed = 0
            if self.gait == "static_walk":
                speed = self.step_length / self.cycle_duration


            # Save in a csv file
            with open(path+'params.csv', 'w+') as f:
                f.write("%s,%s\n" %("robot_name", self.robot_name))
                f.write("%s,%s\n" %("gait", self.gait))
                f.write("%s,%f\n" %("speed", speed))


            # ================================================================ #

            print("DONE")
            print("Saved the CSVs in %s" % os.getcwd() + "/" + path)

            self.destroy_node()



# ============================================================================ #
#                                     main                                     #
# ============================================================================ #

def main(args=None):
    rclpy.init(args=args)

    node = LoggerSubscriber()


    # =============================== Main Loop ============================== #

    rclpy.spin(node)

    rclpy.shutdown()



if __name__ == '__main__':
    main()