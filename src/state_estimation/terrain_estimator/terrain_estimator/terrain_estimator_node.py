import numpy as np
import rclpy
from rclpy.node import Node

from generalized_pose_msgs.msg import GeneralizedPose
from std_msgs.msg import Float64MultiArray



class TerrainEstimator(Node):
    def __init__(self):
        super().__init__("terrain_estimator_node")
        
        # ============================ Subscribers =========================== #
        
        self.desired_generalized_pose_subscription = self.create_subscription(
            GeneralizedPose,
            "robot/desired_generalized_pose",
            self.desired_generalized_pose_callback,
            1)
        
        self.feet_positions = self.create_subscription(
            Float64MultiArray,
            "logging/feet_positions",
            self.feet_positions_callback,
            1)
        
        # ================= Variables Saved By The Subscriber ================ #
        
        self.last_contact_feet_position = np.array([
             1.,  1., 0.,
             1., -1., 0.,
            -1.,  1., 0.,
            -1., -1., 0.   
        ])
        
        self.feet_names = ["LF", "RF", "LH", "RH"]
        
        self.contact_feet_names = []
        
        self.plane_coeff = np.zeros(3)
        
        # ============================= Publisher ============================ #
        
        self._plane_publisher = self.create_publisher(Float64MultiArray, '/state_estimator/terrain', 1)
        
        # =============================== Timer ============================== #
        
        timer_period = 1/25
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    
    def desired_generalized_pose_callback(self, msg):
        self.contact_feet_names = msg.contact_feet
        
    
    def feet_positions_callback(self, msg):
        for i in range(4):
            if self.feet_names[i] in self.contact_feet_names:
                self.last_contact_feet_position[3*i:3*i+3] = msg.data[3*i:3*i+3]
                
    
    def compute_plane_eq(self):
        # Plane equation: z = a x + b y + c
        
        A = np.block([
            self.last_contact_feet_position.reshape((4, 3))[:, 0:2], np.ones((4,1))
        ])
        
        b = self.last_contact_feet_position.reshape((4, 3))[:, 2]
        
        self.plane_coeff = np.linalg.lstsq(A, b, rcond=None)[0]
                
        
    def timer_callback(self):
        msg = Float64MultiArray()
                
        self.compute_plane_eq()
        
        msg.data = self.plane_coeff.tolist()
        
        self._plane_publisher.publish(msg)
        
        

def main(args=None):
    rclpy.init(args=args)
        
    node = TerrainEstimator()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            
    
    
if __name__ == '__main__':
    main()