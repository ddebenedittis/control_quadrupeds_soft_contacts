import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from gazebo_msgs.msg import LinkStates
from generalized_pose_msgs.msg import GeneralizedPosesWithTime
from geometry_msgs.msg import Pose, Twist, TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu

from .kalman_filter import KalmanFilter



# ============================================================================ #
#                               POSEESTIMATORNODE                              #
# ============================================================================ #

class PoseEstimatorNode(Node):

    def __init__(self):
        super().__init__("pose_estimator_node")
        
        # ============================ Subscriber ============================ #
        
        self.joint_states_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            1)

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
        
        self.gen_pose_subscription = self.create_subscription(
            GeneralizedPosesWithTime,
            "/motion_planner/desired_generalized_poses",
            self.gen_pose_callback,
            1)
        
        
        # ========================== Node Parameters ========================= #
        
        self.declare_parameter("robot_name", "")
        
        
        # ============================== Filter ============================== #
        
        self.filter = KalmanFilter(
            robot_name = str(self.get_parameter('robot_name').value)
        )
        
        self.joint_names = self.filter._robot_model.ordered_joint_names
        
        
        # ================= Variables Saved By The Subscriber ================ #

        # Base position and quaternion orientation (q_nb) in inertial frame: [px, py, pz, qw, qx, qy, qz].
        self.q_b = np.zeros(7)
        # Base linear and angular velocity in inertial frame: [vx, vy, vz, omega_x, omega_y, omega_z]
        self.v_b = np.zeros(6)
        
        self.q_j = np.zeros(12)             # joints position
        self.v_j = np.zeros(12)             # joints velocity
        self.a_b = np.array([0.,0.,9.81])   # base acceleration in body frame
        self.w_b = np.array([0.,0.,0.])     # base angular velocity in body frame
        
        self.joint_states_msg = JointState()
        self.joint_states_msg.name = self.joint_names
        
        # names of the feet in contact with the ground
        self.contact_feet_names = ['LF', 'RF', 'LH', 'RH']
        
        self.nanosec = 0                       # time of last message from the imu
        
        
        # ============================= Publisher ============================ #
        
        self._pose_publisher = self.create_publisher(Pose, '/state_estimator/pose', 1)
        self._twist_publisher = self.create_publisher(Twist, '/state_estimator/twist', 1)
        
        self._joint_states_publisher = self.create_publisher(JointState, '/state_estimator/joint_states', 1)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        # =============================== Timer ============================== #

        timer_period = 1/100    # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        
    def joint_states_callback(self, msg: JointState):
        """
        Read, reorder and save the joint states.
        The joints are saved in the following order: LF_(HAA -> HFE -> KFE) -> LH -> RF -> RH
        In addition, save the JointState header to republish it.
        """
                
        for i in range(len(self.joint_names)):
            self.q_j[self.joint_names.index(msg.name[i])] = msg.position[i]
            self.v_j[self.joint_names.index(msg.name[i])] = msg.velocity[i]
            
        self.joint_states_msg.header = msg.header


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
        self.q_b = np.array([pos.x, pos.y, pos.z, orient.w, orient.x, orient.y, orient.z])
        self.v_b = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])


    def imu_callback(self, msg: Imu):
        acc = msg.linear_acceleration
        ang_vel = msg.angular_velocity

        self.a_b = np.array([acc.x, acc.y, acc.z])
        self.w_b = np.array([ang_vel.x, ang_vel.y, ang_vel.z])
        
        self.nanosec = msg.header.stamp.nanosec / 10**9 # + msg.header.stamp.sec
        
        
    def gen_pose_callback(self, msg: GeneralizedPosesWithTime):
        # It is necessary to save only the names of the feet that the planner wants to be in contact with the terrain.
        self.contact_feet_names = msg.generalized_poses_with_time[0].generalized_pose.contact_feet
        
    
    def publish_pose_twist(self, p, q, v, omega):
        # Publish the pose
        pose_msg = Pose()

        pose_msg.position.x = p[0]
        pose_msg.position.y = p[1]
        pose_msg.position.z = p[2]

        pose_msg.orientation.x = q[1]
        pose_msg.orientation.y = q[2]
        pose_msg.orientation.z = q[3]
        pose_msg.orientation.w = q[0]

        self._pose_publisher.publish(pose_msg)

        # Publish the twist
        twist_msg = Twist()

        twist_msg.linear.x = v[0]
        twist_msg.linear.y = v[1]
        twist_msg.linear.z = v[2]
        
        twist_msg.angular.x = omega[0]
        twist_msg.angular.y = omega[1]
        twist_msg.angular.z = omega[2]

        self._twist_publisher.publish(twist_msg)
        
        
    def broadcast_transform(self, p, q):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ground_plane_link'
        
        t.child_frame_id = "state_estimator/" + str(self.filter._robot_model._model.frames[2].name)
        
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
        
    def timer_callback(self):
        self.filter.predict(self.w_b, self.a_b, self.nanosec)
        
        self.filter.fuse_odo(self.q_j, self.v_j, self.contact_feet_names.copy())
        
        omega = self.w_b - self.filter.gyro_bias
        
        self.publish_pose_twist(self.filter.position, self.filter.orientation_as_float_array, self.filter.velocity, omega)
        
        self.broadcast_transform(self.filter.position, self.filter.orientation_as_float_array)
        
        self.joint_states_msg.position = self.q_j.tolist()
        self._joint_states_publisher.publish(self.joint_states_msg)
        
        
        
class blockingNode(Node):
    def __init__(self):
        super().__init__("blocking_node")
        
        self.clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            1)        
                
    def clock_callback(self, msg):
        if (msg.clock.nanosec == 0):
            rclpy.spin_once(self)
        
        

# ============================================================================ #
#                                     main                                     #
# ============================================================================ #

def main(args=None):
    rclpy.init(args=args)
        
    node = PoseEstimatorNode()
        
    node.filter.decimation_factor = 5
    node.filter.position = np.array([0., 0., 0.6])
    node.filter.flag_fuse_odo_pos = True
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