#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf2_ros import TransformBroadcaster


class DroneNode(Node):
    """
    A simulated drone node that:
    - Subscribes to velocity commands
    - Integrates velocity to update position
    - Publishes its current pose
    - Broadcasts its TF transform
    - Can be enabled/disabled via parameter
    """
    
    def __init__(self):
        super().__init__('drone_node')
        
        # Declare parameters
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('x_start', 0.0)
        self.declare_parameter('y_start', 0.0)
        self.declare_parameter('z_start', 1.0)
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.drone_id = self.get_parameter('drone_id').value
        self.enabled = self.get_parameter('enabled').value
        
        # Current state (position and velocity)
        self.x = self.get_parameter('x_start').value
        self.y = self.get_parameter('y_start').value
        self.z = self.get_parameter('z_start').value
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # Create pose publisher (namespace-aware)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'pose',
            10
        )
        
        # Create velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for state update and publishing (10 Hz)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info(
            f'Drone {self.drone_id} initialized at '
            f'({self.x:.2f}, {self.y:.2f}, {self.z:.2f}), '
            f'enabled={self.enabled}'
        )
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands from swarm controller"""
        if self.enabled:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.vz = msg.linear.z
    
    def timer_callback(self):
        """Integrate velocity and publish state"""
        # Check if drone is enabled via parameter (can change at runtime)
        self.enabled = self.get_parameter('enabled').value
        
        if not self.enabled:
            return  # Don't publish if disabled
        
        # Integrate velocity to update position (simple Euler integration)
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z
        pose_msg.pose.orientation.w = 1.0  # No rotation
        
        self.pose_pub.publish(pose_msg)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = f'drone_{self.drone_id}'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
