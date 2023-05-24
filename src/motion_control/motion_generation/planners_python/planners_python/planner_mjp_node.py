import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from generalized_pose_msgs.msg import GeneralizedPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rviz_legged_msgs.msg import Paths
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from velocity_command_msgs.msg import SimpleVelocityCommand

from .planners.planner_mjp import DesiredGeneralizedPose, MotionPlanner

from .utils.quat_math import q_mult
from .utils.fading_filter import FadingFilter

import numpy as np



# ========================= Get_quaternion_from_euler ======================== #

def get_quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Convert the Euler angles to a unit quaternion.

    Args:
        roll (float): The roll (rotation around x-axis) angle in radians.
        pitch (float): The pitch (rotation around y-axis) angle in radians.
        yaw (float): The yaw (rotation around z-axis) angle in radians.

    Returns:
        ndarray: the orientation in quaternion [x,y,z,w] format
    """

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qx, qy, qz, qw])



# ============================================================================ #
#                            MINIMALSUBSCRIBER CLASS                           #
# ============================================================================ #

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")

        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)

        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            self.imu_callback,
            1)

        self.velocity_command_subscription = self.create_subscription(
            SimpleVelocityCommand,
            "/motion_generator/simple_velocity_command",
            self.vel_cmd_callback,
            1)

        self.terrain_plane_subscription = self.create_subscription(
            Float64MultiArray,
            "/state_estimator/terrain_plane",
            self.terrain_callback,
            1)

        self.terrain_penetration_subscription = self.create_subscription(
            Float64MultiArray,
            "state_estimator/terrain_penetration",
            self.terrain_penetration_callback,
            1)


        self.q_b = np.array([])     # base position
        self.v_b = np.array([])     # base velocity
        self.a_b = np.array([])     # base acceleration

        self.velocity_forward = 0.
        self.velocity_lateral = 0.
        self.yaw_rate = 0.

        # The plane is z = a x + b y + c, where self.plane_coeffs = {a, b, c}.
        self.plane_coeffs = np.zeros(3)

        # The penetrations of the four feet into the ground (LF -> RF -> LH -> RH). This value is used to increase the step height.
        self.terrain_penetrations = np.zeros(4)


    # Save the base pose and twist
    def link_states_callback(self, msg: LinkStates):
        # The index of the base must be found by searching which link contains "base" in its name.
        base_id = -1

        for i, name in enumerate(msg.name):
            if "base" in name:
                base_id = i
                break

        # /gazebo/link_states returns the pose and the twist in the inertial or world frame.

        # Extract the base position and orientation (quaternion)
        pos = msg.pose[base_id].position
        orient = msg.pose[base_id].orientation

        # Extract the base linear and angular velocity
        lin = msg.twist[base_id].linear
        ang = msg.twist[base_id].angular

        # Save the base pose and twists as numpy arrays
        self.q_b = np.array([pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w])
        self.v_b = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])


    # Save the base acceleration
    def imu_callback(self, msg: Imu):
        acc = msg.linear_acceleration

        self.a_b = np.array([acc.x, acc.y, acc.z])


    def vel_cmd_callback(self, msg: SimpleVelocityCommand):
        self.velocity_forward = msg.velocity_forward
        self.velocity_lateral = msg.velocity_lateral
        self.yaw_rate = msg.yaw_rate


    def terrain_callback(self, msg: Float64MultiArray):
        self.plane_coeffs = msg.data


    def terrain_penetration_callback(self, msg: Float64MultiArray):
        self.terrain_penetrations = msg.data



# ============================================================================ #
#                            MINIMALPUBLISHER CLASS                            #
# ============================================================================ #

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        self.gen_pose_publisher_ = self.create_publisher(GeneralizedPose, 'motion_planner/desired_generalized_pose', 1)

        self.feet_trajectory_publisher_ = self.create_publisher(Paths, 'rviz/feet_trajectory', 1)

    def publish_desired_gen_pose(self, des_gen_pose: DesiredGeneralizedPose):
        # Initialize and fully populate the desired generalized pose message

        msg = GeneralizedPose()

        msg.base_acc.x = des_gen_pose.base_acc[0]
        msg.base_acc.y = des_gen_pose.base_acc[1]
        msg.base_acc.z = des_gen_pose.base_acc[2]

        msg.base_vel.x = des_gen_pose.base_vel[0]
        msg.base_vel.y = des_gen_pose.base_vel[1]
        msg.base_vel.z = des_gen_pose.base_vel[2]

        msg.base_pos.x = des_gen_pose.base_pos[0]
        msg.base_pos.y = des_gen_pose.base_pos[1]
        msg.base_pos.z = des_gen_pose.base_pos[2]

        msg.base_angvel.x = des_gen_pose.base_angvel[0]
        msg.base_angvel.y = des_gen_pose.base_angvel[1]
        msg.base_angvel.z = des_gen_pose.base_angvel[2]

        msg.base_quat.x = des_gen_pose.base_quat[0]
        msg.base_quat.y = des_gen_pose.base_quat[1]
        msg.base_quat.z = des_gen_pose.base_quat[2]
        msg.base_quat.w = des_gen_pose.base_quat[3]

        msg.feet_acc = des_gen_pose.feet_acc.tolist()
        msg.feet_vel = des_gen_pose.feet_vel.tolist()
        msg.feet_pos = des_gen_pose.feet_pos.tolist()

        msg.contact_feet = des_gen_pose.contact_feet_names


        self.gen_pose_publisher_.publish(msg)


    def publish_feet_trajectory(self, r_s_des: np.ndarray):
        msg = Paths()

        msg.header.frame_id = "ground_plane_link"

        n_paths = len(r_s_des)

        for i in range(n_paths):
            msg.paths.append(Path())

            n_points = np.shape(r_s_des[i])[0]

            for j in range(n_points):
                msg.paths[i].poses.append(PoseStamped())
                msg.paths[i].poses[j].pose.position.x = r_s_des[i][j,0]
                msg.paths[i].poses[j].pose.position.y = r_s_des[i][j,1]
                msg.paths[i].poses[j].pose.position.z = r_s_des[i][j,2]

        self.feet_trajectory_publisher_.publish(msg)



# ============================================================================ #
#                                    PLANNER                                   #
# ============================================================================ #

class Planner(Node):
    def __init__(self):
        super().__init__("Planner")

        self.time = 0

        self.minimal_publisher = MinimalPublisher()

        self.minimal_subscriber = MinimalSubscriber()

        # Instantiate the motion planner
        self.planner = MotionPlanner()
        self.planner.dt = 1. / 200

        self.planner.Ts = 0.2

        self.default_step_height = 0.1
        self.planner.interp.step_height = self.default_step_height

        # Instantiate the fading filter class (used to filter the base acceleration)
        self.filter = FadingFilter()
        self.filter._order = 2
        self.filter._beta = 0.9

        self.zero_time = 1.     # time before the planner starts (no message is published).
        self.init_time = 0.5    # time during which the robot goes to the initial position, before the real planning starts.

        self.p_b_0 = np.zeros(0)
        self.q_b_0 = np.zeros(0)

        self.rate = self.minimal_subscriber.create_rate(int(1 / self.planner.dt))

        timer_period = self.planner.dt    # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('gait')

        self.teleoperate = False
        if str(self.get_parameter('gait').value) == "teleop_walking_trot":
            self.teleoperate = True


    # ========================= Correct_with_terrain ========================= #

    def correct_with_terrain(self, des_gen_pose: DesiredGeneralizedPose):
        """
        Change the desired base and feet height depending on the local terrain height.
        Change the desired roll and pitch angles of the base to align them to the local terrain plane.
        """

        plane_coeffs = self.minimal_subscriber.plane_coeffs

        # Local terrain height.
        delta_h = plane_coeffs[0] * des_gen_pose.base_pos[0] + plane_coeffs[1] * des_gen_pose.base_pos[1] + plane_coeffs[2]

        # Shift the desired base and feet height.
        des_gen_pose.base_pos[2] += delta_h
        des_gen_pose.feet_pos[2::3] += delta_h

        # Align the desired roll and pitch angles of the base to the local terrain plane.
        des_gen_pose.base_quat = get_quaternion_from_euler(np.arctan(plane_coeffs[1]), - np.arctan(plane_coeffs[0]), self.planner.dtheta)


    # ============================ Timer_callback ============================ #

    def timer_callback(self):
        if self.minimal_subscriber.q_b.size == 0 or self.minimal_subscriber.a_b.size == 0 or self.p_b_0.size == 0:
            if self.minimal_subscriber.q_b.size != 0:
                self.p_b_0 = self.minimal_subscriber.q_b[0:3]
                self.q_b_0 = self.minimal_subscriber.q_b[3:7]

                q = self.q_b_0
                dtheta = np.arctan2(
                    2 * (q[3]*q[2] + q[0]*q[1]),
                    1 - 2 * (q[1]*q[1] +  q[2]*q[2])
                )

                self.planner.update_initial_conditions(self.p_b_0, dtheta)

            self.rate.sleep()
        else:
            # Update the step height to take into account the terrain penetration.
            self.planner.interp.step_height = self.default_step_height \
                + 0.5 * np.mean(self.minimal_subscriber.terrain_penetrations)

            # Update the internal timer
            self.time += self.planner.dt

            # Get the base linear quantities (position, velocity and acceleration)
            p_b = self.minimal_subscriber.q_b[0:3]
            q_b = self.minimal_subscriber.q_b[3:7]
            v_b = self.minimal_subscriber.v_b[0:3]
            a_b_meas = self.minimal_subscriber.a_b

            # Rotate the acceleration in the inertial frame
            q_b_conj = np.array([-q_b[0], -q_b[1], -q_b[2], q_b[3]])
            a_b_quat = np.concatenate((a_b_meas, np.array([0])))
            a_b_meas_body = (q_mult(q_mult(q_b, a_b_quat), q_b_conj))[0:3] + np.array([0., 0., -9.81])

            # Filter the acceleration measured with the
            a_b = - self.filter.filter(a_b_meas_body, Ts=self.planner.dt)

            # Horizontal velocity command and yaw rate command
            vel_cmd = np.array([
                self.minimal_subscriber.velocity_forward,
                self.minimal_subscriber.velocity_lateral,
            ])
            yaw_rate_cmd = self.minimal_subscriber.yaw_rate

            # Perform a single iteration of the model predictive control
            if self.time > self.init_time + self.zero_time:
                # Planner output after the initialization phase has finished
                des_gen_pose = self.planner.update(p_b, v_b, a_b, vel_cmd, yaw_rate_cmd)

                self.correct_with_terrain(des_gen_pose)

                self.minimal_publisher.publish_feet_trajectory(self.planner.trajectory_sample_points())
            elif self.time > self.zero_time:
                contactFeet = ['LF', 'RF', 'LH', 'RH']

                plane_coeffs = self.minimal_subscriber.plane_coeffs

                # Local terrain height.
                delta_h = plane_coeffs[0] * 0 + plane_coeffs[1] * 0 + plane_coeffs[2]

                # Base position quantities
                r_b_des, r_b_dot_des, r_b_ddot_des = self.planner.interp._spline(
                    np.array([self.p_b_0[0], self.p_b_0[1], self.p_b_0[2]]),
                    np.array([self.p_b_0[0], self.p_b_0[1], self.planner.zcom + delta_h]),
                    (self.time - self.zero_time)/self.init_time)

                # Base angular quantities
                omega_des = np.zeros(3)
                q_des = self.q_b_0

                # Swing feet position quantities
                r_s_ddot_des = np.array([])
                r_s_dot_des = np.array([])
                r_s_des = np.array([])

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
            else:
                self.p_b_0[2] = self.minimal_subscriber.q_b[2]

                return


            # Publish the desired generalized pose message
            self.minimal_publisher.publish_desired_gen_pose(des_gen_pose)



# ============================================================================ #
#                                     MAIN                                     #
# ============================================================================ #

def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(planner.minimal_subscriber)

    executor.spin()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
