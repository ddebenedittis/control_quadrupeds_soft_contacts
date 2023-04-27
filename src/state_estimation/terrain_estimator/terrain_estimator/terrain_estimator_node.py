from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ContactsState
from generalized_pose_msgs.msg import GeneralizedPose
from std_msgs.msg import Float64MultiArray

from terrain_estimator.penetration_estimator import PenetrationEstimator
from terrain_estimator.plane_estimator import PlaneEstimator



class FirstOrderFadingFilter():
    """
    Low pass filter that estimates a variable x as: \\
    x_new = beta * x_old + (1 - beta) * measurement
    where beta ∈ [0; 1]
    """
    
    def __init__(self) -> None:
        self.beta = 0.5
                
    def filter(self, old_estimate, measurement):
        return self.beta * old_estimate + (1-self.beta) * measurement



class TerrainEstimator(Node):
    """
    Estimate the contact plane using the feet positions and the list of feet in contact with the terrain. \\
    The plane equation is z = a x + b y + c. The plane coefficients are published in the /state_estimator/terrain topic as {a, b, c}.
    """
    
    def __init__(self):
        super().__init__("terrain_estimator_node")
        
        # ================= Variables Saved By The Subscriber ================ #
        
        self.plane_estimator = PlaneEstimator()
        
        self.penetration_estimator = PenetrationEstimator()
        
        # Low pass filter for the feet positions.
        self.filter = FirstOrderFadingFilter()
        self.filter.beta = 0.9
        
        # List of all the feet names
        self.all_feet_names = ["LF", "RF", "LH", "RH"]
        
        # List of feet in contact with the terrain. The foot position is used to 
        self.contact_feet_names = []
        
        # ============================ Subscribers =========================== #
        
        # Used to know which feet are in contact with the terrain.
        self.desired_generalized_pose_subscription = self.create_subscription(
            GeneralizedPose,
            "robot/desired_generalized_pose",
            self.desired_generalized_pose_callback,
            1)
        
        self.feet_positions_subscription = self.create_subscription(
            Float64MultiArray,
            "logging/feet_positions",
            self.feet_positions_callback,
            1)
        
        self.feet_bumper_sensors_subscriptions = []
        for i, foot_name in enumerate(self.all_feet_names):
            self.feet_bumper_sensors_subscriptions.append(self.create_subscription(
                ContactsState,
                "/contact_force_sensors/" + foot_name,
                partial(self.penetration_estimator.update_penetration_i, index=i),
                1))
        
        # ============================= Publisher ============================ #
        
        self._terrain_plane_publisher = self.create_publisher(Float64MultiArray, '/state_estimator/terrain_plane', 1)
        
        self._terrain_penetration_publisher = self.create_publisher(Float64MultiArray, '/state_estimator/terrain_penetration', 1)
        
        # =============================== Timer ============================== #
        
        # The plane coefficients are published with this period.
        timer_period = 1/25
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    
    def desired_generalized_pose_callback(self, msg):
        self.contact_feet_names = msg.contact_feet
        
    
    def feet_positions_callback(self, msg):
        for i in range(4):
            if self.all_feet_names[i] in self.contact_feet_names:
                # Filter the measurement only if the feet is among the feet in contact with the terrain.
                
                self.plane_estimator.contact_feet_positions[3*i:3*i+3] = self.filter.filter(
                    self.plane_estimator.contact_feet_positions[3*i:3*i+3],
                    np.array([msg.data[3*i:3*i+3]])
                )

    
    def timer_callback(self):
        # Publish the estimated terrain plane coefficients.
        
        # The plane is z = a x + b y + c, where self.plane_coeffs = {a, b, c}.
        plane_coeffs = self.plane_estimator.compute_plane_eq()
        
        msg = Float64MultiArray()
        msg.data = plane_coeffs.tolist()
        
        self._terrain_plane_publisher.publish(msg)
        
        # Publish the estimated terrain penetrations of the four feet.
        
        penetration_msg = Float64MultiArray()
        penetration_msg.data = self.penetration_estimator.get_maximum_penetration().tolist()
        
        self._terrain_penetration_publisher.publish(penetration_msg)
        
        

def main(args=None):
    rclpy.init(args=args)
        
    node = TerrainEstimator()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            
    
    
if __name__ == '__main__':
    main()
