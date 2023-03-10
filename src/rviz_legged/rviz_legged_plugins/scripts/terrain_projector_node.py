#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray



class ZMPCalculator(Node):
    def __init__(self):
        super().__init__('zmp_calculator')
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self.imu_callback,
            1)
        
        self.forces_subscription = self.create_subscription(
            Float64MultiArray,
            '/logging/optimal_forces',
            self.forces_callback,
            1)
        
        self.feet_positions_subscription = self.create_subscription(
            Float64MultiArray,
            '/logging/feet_positions',
            self.feet_positions_callback,
            1)
        
        self.a_b = np.zeros(3)
        
        self.forces = np.zeros((4,3))
        
        self.feet_positions = np.zeros((4,3))
                
        
    def imu_callback(self, msg):
        self.a_b = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        
    def forces_callback(self, msg):
        self.forces = np.array(msg.data).reshape(4,3)
                
        
    def feet_positions_callback(self, msg):
        self.feet_positions = np.array(msg.data).reshape(4,3)
        
        
    def compute_zmp(self, com, normal):
        F_c = np.zeros(3)
        M_c = np.zeros(3)
        for i in range(4):
            F_c = F_c + self.forces[i,:]
            M_c = M_c + np.cross((com - self.feet_positions[i,:]), self.forces[i,:])
            
        if np.dot(F_c, normal) != 0:
            com_2_zmp = np.cross(normal, M_c) / np.dot(F_c, normal)
        else:
            com_2_zmp = np.zeros(3)
        
        return com + com_2_zmp



class TerrainProjector(Node):

    def __init__(self):
        super().__init__('terrain_projector_node')
        
        # ============================ Subscribers =========================== #

        self.terrain_subscription = self.create_subscription(
            Float64MultiArray,
            '/state_estimator/terrain',
            self.terrain_callback,
            1)
        
        self.com_position_subscription = self.create_subscription(
            PointStamped,
            '/rviz/com_position',
            self.com_position_callback,
            1)
        
        # ============================= Publisher ============================ #
        
        self.projected_com_position_publisher = self.create_publisher(PointStamped, '/rviz/projected_com_position', 1)
        
        self.projected_zmp_position_publisher = self.create_publisher(PointStamped, '/rviz/projected_zmp_position', 1)
        
        # =============================== Timer ============================== #
        
        timer_period = 1/25
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # ========================== Saved Variables ========================= #
        
        self.plane_coeffs = np.array([0., 0., 1., 0.])
        
        self.com_position = np.zeros(3)
        
        self.projected_com_position = np.zeros(3)
        
        self.zmp_calculator = ZMPCalculator()


    # Compute the coefficients of the plane from the published coefficients.
    def terrain_callback(self, msg):
        #The plane is in this form: z = a x + b y + c
        old_plane_coeffs = np.array(msg.data)
        
        # Given a plane a x + b y + c z = d, the plane normal is (a, b, c).
        plane_normal = np.array([- old_plane_coeffs[0], - old_plane_coeffs[1], 1.])
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        
        # The plane is in this form: a x + b y + c z = d
        self.plane_coeffs = np.concatenate((
            plane_normal,
            np.array([old_plane_coeffs[2] / np.linalg.norm(plane_normal)])
        ))
        
        
    def com_position_callback(self, msg):
        self.com_position = np.array([msg.point.x, msg.point.y, msg.point.z])        
        
        
    # Publish the projection of the com position on the terrain plane.
    def timer_callback(self):
        a = self.plane_coeffs[0]
        b = self.plane_coeffs[1]
        c = self.plane_coeffs[2]
        d = self.plane_coeffs[3]
        
        k = (d - a*self.com_position[0] - b*self.com_position[1] - c*self.com_position[2]) / (a**2 + b**2 + c**2)
        
        self.projected_com_position = self.com_position + k * np.array([a, b, c])
        
        msg = PointStamped()
        
        msg.header.frame_id = "ground_plane_link"
        
        msg.point.x = self.projected_com_position[0]
        msg.point.y = self.projected_com_position[1]
        msg.point.z = self.projected_com_position[2]
        
        self.projected_com_position_publisher.publish(msg)
        
        # ==================================================================== #
        
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)
        zmp = self.zmp_calculator.compute_zmp(self.projected_com_position, normal)
        
        msg = PointStamped()
        
        msg.header.frame_id = "ground_plane_link"
        
        msg.point.x = zmp[0]
        msg.point.y = zmp[1]
        msg.point.z = zmp[2]
        
        self.projected_zmp_position_publisher.publish(msg)
        
        

def main():
    rclpy.init()
    terrain_projector_node = TerrainProjector()
    
    executor = MultiThreadedExecutor()
    executor.add_node(terrain_projector_node)
    executor.add_node(terrain_projector_node.zmp_calculator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()