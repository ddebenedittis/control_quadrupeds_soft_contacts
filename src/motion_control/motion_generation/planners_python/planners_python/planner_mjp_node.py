import threading
import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu
from generalized_pose_msgs.msg import GeneralizedPose

from .planners.planner_mjp import MotionPlanner

from .utils.quat_math import q_mult
from .utils.fading_filter import FadingFilter

import numpy as np



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
        
        self.link_states_subscription # prevent unused variable warning

        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            self.imu_callback,
            1)

        self.imu_subscription # prevent unused variable warning

        self.q_b = np.array([])     # base position
        self.v_b = np.array([])     # base velocity
        self.a_b = np.array([])     # base acceleration


    # Save the base pose and twist
    def link_states_callback(self, msg):
        # The index of the base must be found by searching which link contains "base" in its name.
        base_id = -1
        
        for i in range(len(msg.name)):
            if "base" in msg.name[i]:
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
    def imu_callback(self, msg):
        acc = msg.linear_acceleration

        self.a_b = np.array([acc.x, acc.y, acc.z])



# ============================================================================ #
#                            MINIMALPUBLISHER CLASS                            #
# ============================================================================ #

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        self.publisher_ = self.create_publisher(GeneralizedPose, 'robot/desired_generalized_pose', 1)

    def publish_desired_gen_pose(
        self,
        contactFeet,
        r_b_ddot_des, r_b_dot_des, r_b_des,
        omega_des, q_des,
        r_s_ddot_des, r_s_dot_des, r_s_des
    ):
        # Initialize and fully populate the desired generalized pose message

        msg = GeneralizedPose()

        msg.base_acc.x = r_b_ddot_des[0]
        msg.base_acc.y = r_b_ddot_des[1]
        msg.base_acc.z = r_b_ddot_des[2]

        msg.base_vel.x = r_b_dot_des[0]
        msg.base_vel.y = r_b_dot_des[1]
        msg.base_vel.z = r_b_dot_des[2]

        msg.base_pos.x = r_b_des[0]
        msg.base_pos.y = r_b_des[1]
        msg.base_pos.z = r_b_des[2]

        msg.base_angvel.x = omega_des[0]
        msg.base_angvel.y = omega_des[1]
        msg.base_angvel.z = omega_des[2]

        msg.base_quat.x = q_des[0]
        msg.base_quat.y = q_des[1]
        msg.base_quat.z = q_des[2]
        msg.base_quat.w = q_des[3]

        msg.feet_acc = r_s_ddot_des.tolist()
        msg.feet_vel = r_s_dot_des.tolist()
        msg.feet_pos = r_s_des.tolist()

        msg.contact_feet = contactFeet


        self.publisher_.publish(msg)
        
        
        
class Planner(Node):
    def __init__(self):
        super().__init__("Planner")
        
        self.time = 0
        
        self.minimal_publisher = MinimalPublisher()
        
        self.minimal_subscriber = MinimalSubscriber()
        
        # Instantiate the motion planner
        self.planner = MotionPlanner()
        self.planner.dt = 1. / 200
        
        # Instantiate the fading filter class (used to filter the base acceleration)
        self.filter = FadingFilter()
        self.filter.order = 2
        self.filter.beta = 0.5
        
        self.zero_time = 1.     # time before the planner starts (no message is published).
        self.init_time = 0.5    # time during which the robot goes to the initial position, before the real planning starts.
        
        self.p_b_0 = np.array([0., 0., 0.])
        
        self.rate = self.minimal_subscriber.create_rate(int(1 / self.planner.dt))
        
        timer_period = self.planner.dt    # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
    def timer_callback(self):        
        if self.minimal_subscriber.q_b.size == 0 or self.minimal_subscriber.a_b.size == 0:
            if self.minimal_subscriber.q_b.size != 0:
                self.p_b_0 = self.minimal_subscriber.q_b[0:3]
                
            self.rate.sleep()
        else:
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
            # vel_cmd = np.array([0.0,0.0])
            vel_cmd = - np.array([
                p_b[0],
                p_b[1]
            ])
            yaw_rate_cmd = 0

            # Perform a single iteration of the model predictive control
            if self.time > self.init_time + self.zero_time:
                # Planner output after the initialization phase has finished
                contactFeet, r_b_ddot_des, r_b_dot_des, r_b_des, omega_des, q_des, r_s_ddot_des, r_s_dot_des, r_s_des = self.planner.mpc(p_b, v_b, a_b, vel_cmd, yaw_rate_cmd)
            elif self.time > self.zero_time:
                contactFeet = ['LF', 'RF', 'LH', 'RH']

                # Base position quantities
                r_b_des, r_b_dot_des, r_b_ddot_des = self.planner._spline(np.array([self.p_b_0[0], self.p_b_0[1], self.p_b_0[2]]), np.array([self.p_b_0[0], self.p_b_0[1], self.planner.zcom]), (self.time - self.zero_time)/self.init_time)

                # Base angular quantities
                omega_des = np.zeros(3)
                q_des = np.array([0., 0., 0., 1.])
            
                # Swing feet position quantities
                r_s_ddot_des = np.array([])
                r_s_dot_des = np.array([])
                r_s_des = np.array([])
            else:
                self.p_b_0[2] = self.minimal_subscriber.q_b[2]
                
                return
            

            # Publish the desired generalized pose message
            self.minimal_publisher.publish_desired_gen_pose(
                contactFeet,
                r_b_ddot_des, r_b_dot_des, r_b_des,
                omega_des, q_des,
                r_s_ddot_des, r_s_dot_des, r_s_des
            )



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