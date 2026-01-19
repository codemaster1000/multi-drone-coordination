#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the multi-drone sandbox.
    
    Launches:
    - 3 drone nodes (each in their own namespace)
    - 1 supervisor node
    """
    
    return LaunchDescription([
        # Drone 1 - positioned at origin
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_1',
            name='drone_node',
            parameters=[{
                'drone_id': 1,
                'x_offset': 0.0,
                'y_offset': 0.0,
                'z_offset': 0.0,
            }],
            output='screen',
        ),
        
        # Drone 2 - positioned 5 units to the right
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_2',
            name='drone_node',
            parameters=[{
                'drone_id': 2,
                'x_offset': 5.0,
                'y_offset': 0.0,
                'z_offset': 0.0,
            }],
            output='screen',
        ),
        
        # Drone 3 - positioned 5 units forward
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_3',
            name='drone_node',
            parameters=[{
                'drone_id': 3,
                'x_offset': 0.0,
                'y_offset': 5.0,
                'z_offset': 0.0,
            }],
            output='screen',
        ),
        
        # Supervisor node - monitors all drones
        Node(
            package='multi_drone_pkg',
            executable='supervisor',
            name='supervisor',
            output='screen',
        ),
    ])
