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

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ground_plane_link'
        t.child_frame_id = 'base'
        
        base_id = -1
        
        for i in range(len(msg.name)):
            if "base" in msg.name[i]:
                base_id = i
                break
            
        t.transform.translation.x = msg.pose[base_id].position.x
        t.transform.translation.y = msg.pose[base_id].position.y
        t.transform.translation.z = msg.pose[base_id].position.z

        t.transform.rotation.x = msg.pose[base_id].orientation.x
        t.transform.rotation.y = msg.pose[base_id].orientation.y
        t.transform.rotation.z = msg.pose[base_id].orientation.z
        t.transform.rotation.w = msg.pose[base_id].orientation.w

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