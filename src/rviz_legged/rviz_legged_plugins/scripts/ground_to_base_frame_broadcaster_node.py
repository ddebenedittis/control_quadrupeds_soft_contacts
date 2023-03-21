#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import LinkStates

from tf2_ros import TransformBroadcaster



class FramePublisher(Node):

    def __init__(self):
        super().__init__('ground_to_base_frame_broadcaster')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.handle_base_pose,
            1)
        self.subscription  # prevent unused variable warning

    def handle_base_pose(self, msg):        
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ground_plane_link'
        
        entities = ["ground_plane"]
        
        for i in range(len(msg.name)):
            entity_name = msg.name[i].split("::")[0]
            
            if entity_name not in entities:
                entities.append(entity_name)
                
                t.child_frame_id = msg.name[i].split("::")[1]
                
                t.transform.translation.x = msg.pose[i].position.x
                t.transform.translation.y = msg.pose[i].position.y
                t.transform.translation.z = msg.pose[i].position.z

                t.transform.rotation.x = msg.pose[i].orientation.x
                t.transform.rotation.y = msg.pose[i].orientation.y
                t.transform.rotation.z = msg.pose[i].orientation.z
                t.transform.rotation.w = msg.pose[i].orientation.w

                # Send the transformation
                self.tf_broadcaster.sendTransform(t)



def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()