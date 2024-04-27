import numpy as np

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from generalized_pose_msgs.msg import GeneralizedPose

from .planners.planner_mjp import DesiredGeneralizedPose



class DebugPlanner(Node):
    def __init__(self):
        super().__init__("debug_planner")
        
        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)
        
        self.gen_pose_publisher_ = self.create_publisher(
            GeneralizedPose, 'motion_planner/desired_generalized_pose', 1
        )
        
        self.q_b = np.array([])
        self.v_b = np.array([])
        
        timer_period = 1. / 200
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
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
        
    def publish_desired_gen_pose(self):
        # Initialize and fully populate the desired generalized pose message

        msg = GeneralizedPose()

        msg.base_acc.x = 0.
        msg.base_acc.y = 0.
        msg.base_acc.z = 0.

        msg.base_vel.x = 0.
        msg.base_vel.y = 0.
        msg.base_vel.z = 0.

        msg.base_pos.x = self.q_b[0]
        msg.base_pos.y = self.q_b[1]
        msg.base_pos.z = self.q_b[2]

        msg.base_angvel.x = 0.
        msg.base_angvel.y = 0.
        msg.base_angvel.z = 0.

        msg.base_quat.x = self.q_b[3]
        msg.base_quat.y = self.q_b[4]
        msg.base_quat.z = self.q_b[5]
        msg.base_quat.w = self.q_b[6]

        msg.feet_acc = np.zeros(3).tolist()
        msg.feet_vel = np.zeros(3).tolist()
        msg.feet_pos = np.array([
            self.q_b[0] + 0.1, self.q_b[1] + 0.05, self.q_b[2] - 0.25,
            # self.q_b[0] + 0.1, self.q_b[1] - 0.05, self.q_b[2] - 0.3,
            # self.q_b[0] - 0.1, self.q_b[1] + 0.05, self.q_b[2] - 0.35,
            # self.q_b[0] - 0.1, self.q_b[1] - 0.05, self.q_b[2] - 0.2,
        ]).tolist()

        msg.contact_feet = ['RF', 'LH', 'RH']
        # msg.contact_feet = ['LF', 'RF', 'LH', 'RH']


        self.gen_pose_publisher_.publish(msg)
        
    def timer_callback(self):        
        if len(self.q_b) == 7:
            self.publish_desired_gen_pose()
        


def main(args=None):
    rclpy.init(args=args)

    debug_planner = DebugPlanner()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(debug_planner)

    executor.spin()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
