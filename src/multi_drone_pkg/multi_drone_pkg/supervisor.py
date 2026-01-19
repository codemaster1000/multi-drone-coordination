#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SupervisorNode(Node):
    """
    A supervisor node that:
    - Subscribes to all 3 drone poses
    - Monitors and logs their positions
    """
    
    def __init__(self):
        super().__init__('supervisor')
        
        # Dictionary to store latest poses
        self.drone_poses = {}
        
        # Subscribe to each drone's pose
        self.drone_1_sub = self.create_subscription(
            PoseStamped,
            '/drone_1/pose',
            lambda msg: self.pose_callback(msg, 1),
            10
        )
        
        self.drone_2_sub = self.create_subscription(
            PoseStamped,
            '/drone_2/pose',
            lambda msg: self.pose_callback(msg, 2),
            10
        )
        
        self.drone_3_sub = self.create_subscription(
            PoseStamped,
            '/drone_3/pose',
            lambda msg: self.pose_callback(msg, 3),
            10
        )
        
        # Timer for periodic status reports (2 Hz)
        self.timer = self.create_timer(0.5, self.report_status)
        
        self.get_logger().info('Supervisor initialized, monitoring 3 drones')
    
    def pose_callback(self, msg: PoseStamped, drone_id: int):
        """Store the latest pose for each drone"""
        self.drone_poses[drone_id] = msg.pose
    
    def report_status(self):
        """Periodically report the status of all drones"""
        if not self.drone_poses:
            return
        
        status_lines = ['=== Drone Status ===']
        for drone_id in sorted(self.drone_poses.keys()):
            pose = self.drone_poses[drone_id]
            status_lines.append(
                f'Drone {drone_id}: '
                f'x={pose.position.x:.2f}, '
                f'y={pose.position.y:.2f}, '
                f'z={pose.position.z:.2f}'
            )
        
        self.get_logger().info('\n'.join(status_lines))


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
