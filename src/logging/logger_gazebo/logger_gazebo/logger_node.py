import numpy as np
import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from generalized_pose_msgs.msg import GeneralizedPose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray



# Quaternion distance
def q_dist(q1, q2):
    
    theta = np.arccos(2 * np.dot(q1, q2) - 1)

    return theta



# ============================================================================ #
#                               POSEESTIMATORNODE                              #
# ============================================================================ #

class LoggerSubscriber(Node):
    """
    
    """

    def __init__(self):
        super().__init__("logger_subscriber")
        
        # ============================ Subscribers =========================== #
        
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
        
        self.feet_position_subscription = self.create_subscription(
            Float64MultiArray,
            "/logging/feet_position",
            self.feet_position_callback,
            1)

        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)
        self.link_states_subscription   # prevent unused variable warning
        
        self.gen_pose_subscription = self.create_subscription(
            GeneralizedPose,
            "/robot/desired_generalized_pose",
            self.gen_pose_callback,
            1)
        self.gen_pose_subscription      # prevent unused variable warning
        
        
        # ================ Variables Saved By The Subscribers ================ #
        
        self.optimal_torques = np.zeros(12)
        self.optimal_forces = np.zeros(12)
        self.optimal_deformations = np.zeros(12)
        self.feet_position = np.zeros((3,4))

        self.q_b = np.zeros(7); self.q_b[6] = 1
        self.v_b = np.zeros(6)
        
        self.desired_base_pos = np.zeros(3)
        self.desired_base_quat = np.zeros(4); self.desired_base_quat[3] = 1
        self.desired_base_vel = np.zeros(3)
        self.contact_feet_names = []
        
        self.time = 0.                      # time of last message from clock
        
        
        # =============================== Timer ============================== #
        
        timer_period = 1/25
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        # ============================= Slippage ============================= #
        
        # Generic names of the feet in contact with the ground
        self.generic_feet_names = ['LF', 'RF', 'LH', 'RH']
        
        # Matrix containing the positions of the feet in contact with the terrain. The feet are supposed to be in contact with the terrain when the planner commands them to be in contact with the terrain.
        # TODO: use contact forces informations to compute this quantity.
        self.saved_feet_pos = np.full((3,4), np.nan)
        
        #
        self.threshold = 0.001
        
        
        # ============================= Csv Data ============================= #
        
        time_horizon = 10
        self.n = int(time_horizon / timer_period + 1)
        self.i = 0
        
        self.time_vector = np.zeros(self.n)
        
        self.optimal_torques_vector = np.zeros((self.n, 12))
        self.optimal_forces_vector = np.zeros((self.n, 12))
        self.optimal_deformations_vector = np.zeros((self.n, 12))
        self.feet_position_vector = np.zeros((self.n, 12))
        
        self.forces_vector = np.zeros((self.n, 12))
        self.deformations_vector = np.zeros(self.n)
        
        self.position_error_vector = np.zeros((self.n, 3))
        self.orientation_error_vector = np.zeros(self.n)
        self.velocity_error_vector = np.zeros((self.n, 3))
        
        self.slippage_vector = np.zeros(self.n)


    def optimal_torques_callback(self, msg):
        self.optimal_torques = np.array(msg.data)
        
        
    def optimal_forces_callback(self, msg):
        self.optimal_forces = np.array(msg.data)
        
        
    def optimal_deformations_callback(self, msg):
        self.optimal_deformations = np.array(msg.data)
        
        
    def feet_position_callback(self, msg):
        self.feet_position = np.array(msg.data).reshape((4,3)).T


    def link_states_callback(self, msg):
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
        self.q_b = np.array([pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w])
        self.v_b = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])
        
        
    def gen_pose_callback(self, msg):
        self.desired_base_pos = np.array([msg.base_pos.x, msg.base_pos.y, msg.base_pos.z])
        self.desired_base_quat = np.array([msg.base_quat.x, msg.base_quat.y, msg.base_quat.z, msg.base_quat.w])
        self.desired_base_vel = np.array([msg.base_vel.x, msg.base_vel.y, msg.base_vel.z])
                
        self.contact_feet_names = msg.contact_feet
        
        
    def compute_slippage(self, contact_feet_names, feet_pos):
        
        # Shortened and ordered feet names
        feet = self.generic_feet_names
        
        # Initialize the slippage at this time step
        slippage = 0
        
        for i in range(4):
            if feet[i] in contact_feet_names:
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
                                
        return slippage
        
    
    def timer_callback(self):
        self.i += 1
        
        self.time_vector[self.i] = self.get_clock().now().nanoseconds / 10**9
        self.position_error_vector[self.i,:] = self.q_b[0:3] - self.desired_base_pos
        self.orientation_error_vector[self.i] = q_dist(self.q_b[3:7], self.desired_base_quat)
        self.velocity_error_vector[self.i,:] = self.v_b[0:3] - self.desired_base_vel
        self.optimal_torques_vector[self.i,:] = self.optimal_torques
        self.optimal_forces_vector[self.i,:] = self.optimal_forces
        self.optimal_deformations_vector[self.i,:] = self.optimal_deformations
        self.slippage_vector[self.i] = self.slippage_vector[self.i-1] + self.compute_slippage(self.contact_feet_names, self.feet_position)
        
        
        if self.i >= self.n - 1:
            path = "/home/davide/Dropbox/phd/quadrupeds/control_quadrupeds_soft_contacts/log/"
            
            np.savetxt(path+"time.csv", self.time_vector, delimiter=",")
            np.savetxt(path+"position_error.csv", self.position_error_vector, delimiter=",")
            np.savetxt(path+"orientation_error.csv", self.orientation_error_vector, delimiter=",")
            np.savetxt(path+"velocity_error.csv", self.velocity_error_vector, delimiter=",")
            np.savetxt(path+"optimal_torques.csv", self.optimal_torques_vector, delimiter=",")
            np.savetxt(path+"optimal_forces.csv", self.optimal_forces_vector, delimiter=",")
            np.savetxt(path+"optimal_deformations.csv", self.optimal_deformations_vector, delimiter=",")
            np.savetxt(path+"slippage.csv", self.slippage_vector, delimiter=",")
            
            print("DONE")
            
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