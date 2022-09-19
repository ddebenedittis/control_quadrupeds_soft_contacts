from math import asin, atan2
import threading

import numpy as np
import quaternion

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from generalized_pose_msgs.msg import GeneralizedPose
from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu

from .kalman_filter import KalmanFilter



# ============================================================================ #
#                               POSEESTIMATORNODE                              #
# ============================================================================ #

class PoseEstimatorNode(Node):
    """
    
    """

    def __init__(self, robot_name):
        super().__init__("pose_estimator_node")
        
        # ============================ Subscriber ============================ #
        
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            1)        
        self.joint_state_subscription   # prevent unused variable warning

        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)
        self.link_states_subscription   # prevent unused variable warning

        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            self.imu_callback,
            1)
        self.imu_subscription           # prevent unused variable warning
        
        self.gen_pose_subscription = self.create_subscription(
            GeneralizedPose,
            "/robot/desired_generalized_pose",
            self.gen_pose_callback,
            1)
        self.gen_pose_subscription      # prevent unused variable warning
        
        
        # ================= Variables Saved By The Subscriber ================ #

        self.q_b = np.zeros(7)
        self.v_b = np.zeros(6)
        
        self.q_j = np.zeros(12)         # joints position
        self.v_j = np.zeros(12)         # joints velocity
        self.a_b = np.array([])         # base acceleration in body frame
        self.w_b = np.array([])         # base angular velocity in body frame
        
        self.contact_feet_names = []    # names of the feet in contact with the ground
        
        self.time = 0                   # time of last message from the imu
        
        
        # ============================= Publisher ============================ #
        
        self._pose_publisher = self.create_publisher(Pose, '/state_estimator/pose', 1)
        self._twist_publisher = self.create_publisher(Twist, '/state_estimator/twist', 1)
        
        
        # =============================== Timer ============================== #

        timer_period = 1/100    # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        # ============================== Filter ============================== #
        
        self.filter = KalmanFilter(robot_name)
        
        
        
    def joint_state_callback(self, msg):
        """
        Read, reorder and save the joint states.
        The joints are saved in the following order: LF_(HAA -> HFE -> KFE) -> LH -> RF -> RH,
        """

        joint_names = ["LF_HAA", "LF_HFE", "LF_KFE",
                       "LH_HAA", "LH_HFE", "LH_KFE",
                       "RF_HAA", "RF_HFE", "RF_KFE",
                       "RH_HAA", "RH_HFE", "RH_KFE"]
        
        for i in range(len(joint_names)):
            self.q_j[joint_names.index(msg.name[i])] = msg.position[i]
            self.v_j[joint_names.index(msg.name[i])] = msg.velocity[i]


    def link_states_callback(self, msg):
        # The index [1] is used because the first link ([0]) is the ground_plane. The second one (the index [1]) corresponds to anymal base.
        base_id = 1

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


    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        ang_vel = msg.angular_velocity

        self.a_b = np.array([acc.x, acc.y, acc.z])
        self.w_b = np.array([ang_vel.x, ang_vel.y, ang_vel.z])
        
        self.time = msg.header.stamp.nanosec / 10**9
        
        
    def gen_pose_callback(self, msg):
        # It is necessary to save only the names of the feet that the planner wants to be in contact with the terrain.
        self.contact_feet_names = msg.contact_feet
        
    
    def publish_pose_twist(self, p, q, v):
        # Publish the pose
        pose_msg = Pose()

        pose_msg.position.x = p[0]
        pose_msg.position.y = p[1]
        pose_msg.position.z = p[2]

        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self._pose_publisher.publish(pose_msg)

        # Publish the twist
        twist_msg = Twist()

        twist_msg.linear.x = v[0]
        twist_msg.linear.y = v[1]
        twist_msg.linear.z = v[2]

        self._twist_publisher.publish(twist_msg)
        
        
    def timer_callback(self):
        self.filter.predict(self.w_b, self.a_b, self.time)

        self.filter.fuse_odo(self.q_j, self.v_j, self.contact_feet_names.copy())
        
        self.publish_pose_twist(self.filter._state[10:13], self.filter._state[0:4], self.filter._state[13:16])
        
        # Get the yaw, pitch, and roll angles from the orientation quaternion and print them.
        # q = quaternion.from_float_array(filter._state[0:4])
        # yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        # pitch = asin(-2.0*(q.x*q.z - q.w*q.y))
        # roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
        # print(roll, " - ", pitch, " - ", yaw)
        
        
        
class blockingNode(Node):
    def __init__(self):
        super().__init__("blocking_node")
        
        self.clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            1)        
        self.clock_subscription   # prevent unused variable warning
                
    def clock_callback(self, msg):
        if (msg.clock.nanosec == 0):
            rclpy.spin_once(self)
        
        

# ============================================================================ #
#                                     main                                     #
# ============================================================================ #

def main(args=None):
    rclpy.init(args=args)
        
    node = PoseEstimatorNode("anymal_c")
        
    node.filter.decimation_factor = 5
    node.filter._state[12] = 0.6
    node.filter._flag_fuse_odo_pos = True
    node.filter.update_private_properties()
        
    
    # =================== Block Until The Simulation Starts ================== #

    # While cycle used to pause the execution of the script until the simulation is started and thus the measurements are obtained.
    blocking_node = blockingNode()
    rclpy.spin_once(blocking_node)
    blocking_node.destroy_node()
            
        
    # =============================== Main Loop ============================== #
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            
    
    
if __name__ == '__main__':
    main()