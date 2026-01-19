#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the intelligent swarm system.
    
    Launches:
    - 3 drone nodes (velocity-controlled)
    - 1 swarm controller (centralized boids)
    
    Supports FLOCK and FOLLOW modes
    """
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='FLOCK',
        description='Swarm mode: FLOCK or FOLLOW'
    )
    
    # Common swarm controller parameters
    swarm_params = {
        # Mode (can be overridden from command line)
        'mode': LaunchConfiguration('mode'),
        'num_drones': 5,
        
        # Boids weights
        'separation_weight': 1.5,
        'cohesion_weight': 0.3,  # Reduced to allow more drift
        'alignment_weight': 1.2,  # Increased to maintain group velocity
        
        # Distance thresholds
        'separation_distance': 2.0,
        'neighbor_distance': 10.0,
        
        # Motion constraints
        'max_speed': 2.0,
        'max_acceleration': 1.0,
        
        # Leader config
        'leader_id': 1,
        'leader_speed': 1.0,  # Increased for more visible movement
        'leader_radius': 4.0,  # Larger circle
        
        # Health monitoring
        'timeout_seconds': 1.0,
        
        # Collision avoidance
        'collision_threshold': 0.5,
        'collision_weight': 10.0,
        
        # Bias velocity (gives flock a direction - like wind)
        'bias_velocity_x': 0.8,
        'bias_velocity_y': 0.5,
        'bias_velocity_z': 0.0,
    }
    
    return LaunchDescription([
        # Launch argument
        mode_arg,
        
        # Drone 1 - Center
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_1',
            name='drone_node',
            parameters=[{
                'drone_id': 1,
                'x_start': 0.0,
                'y_start': 0.0,
                'z_start': 1.0,
                'enabled': True,
            }],
            output='screen',
        ),
        
        # Drone 2 - Right
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_2',
            name='drone_node',
            parameters=[{
                'drone_id': 2,
                'x_start': 3.0,
                'y_start': 0.0,
                'z_start': 1.0,
                'enabled': True,
            }],
            output='screen',
        ),
        
        # Drone 3 - Top
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_3',
            name='drone_node',
            parameters=[{
                'drone_id': 3,
                'x_start': 0.0,
                'y_start': 3.0,
                'z_start': 1.0,
                'enabled': True,
            }],
            output='screen',
        ),
        
        # Drone 4 - Bottom-left
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_4',
            name='drone_node',
            parameters=[{
                'drone_id': 4,
                'x_start': -2.5,
                'y_start': -2.0,
                'z_start': 1.0,
                'enabled': True,
            }],
            output='screen',
        ),
        
        # Drone 5 - Top-right
        Node(
            package='multi_drone_pkg',
            executable='drone_node',
            namespace='drone_5',
            name='drone_node',
            parameters=[{
                'drone_id': 5,
                'x_start': 3.0,
                'y_start': 3.0,
                'z_start': 1.0,
                'enabled': True,
            }],
            output='screen',
        ),
        
        # Swarm Controller (centralized)
        Node(
            package='multi_drone_pkg',
            executable='swarm_controller',
            name='swarm_controller',
            parameters=[swarm_params],
            output='screen',
        ),
    ])
