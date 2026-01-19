#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    RViz-only launch file for swarm visualization (no Gazebo)

    Launches:
    - 5 simple drone nodes (kinematic simulation)
    - 1 swarm controller (Phase 2 with state machine)
    - RViz2 for visualization

    This is lightweight and perfect for:
    - Algorithm development
    - Swarm behavior testing
    - Running without GPU
    """

    # Swarm controller parameters
    swarm_params = {
        # Safety
        "collision_threshold": 0.3,  # Reduced to avoid false collision alarms
        "collision_weight": 10.0,
        "timeout_seconds": 1.0,
        "leader_lost_timeout": 2.0,
        "min_active_drones": 1,
        # Behavior
        "separation_weight": 1.5,
        "cohesion_weight": 1.0,
        "alignment_weight": 1.0,
        "separation_distance": 2.0,
        "neighbor_distance": 15.0,  # Increased so drones detect each other
        "bias_velocity_x": 0.0,  # Remove bias for circular motion
        "bias_velocity_y": 0.0,
        "bias_velocity_z": 0.0,
        # Limits
        "max_speed": 2.0,
        "max_acceleration": 1.0,
        "safe_mode_speed": 0.5,
        # Config
        "num_drones": 5,
        "leader_id": 1,
        "leader_speed": 1.0,  # Increased for visible circular motion
        "leader_radius": 5.0,  # Circle radius
    }

    # Drone starting positions (pentagon formation)
    drone_positions = [
        {"x": 0.0, "y": 0.0, "z": 1.0},  # Center
        {"x": 3.0, "y": 0.0, "z": 1.0},  # East
        {"x": 0.9, "y": 2.9, "z": 1.0},  # North-East
        {"x": -2.4, "y": 1.8, "z": 1.0},  # North-West
        {"x": -2.4, "y": -1.8, "z": 1.0},  # South-West
    ]

    # Create drone nodes
    drone_nodes = []
    for i in range(5):
        drone_nodes.append(
            Node(
                package="multi_drone_pkg",
                executable="drone_node",
                namespace=f"drone_{i+1}",
                name="drone_node",
                parameters=[
                    {
                        "drone_id": i + 1,
                        "x_start": drone_positions[i]["x"],
                        "y_start": drone_positions[i]["y"],
                        "z_start": drone_positions[i]["z"],
                        "enabled": True,
                    }
                ],
                output="screen",
            )
        )

    # Swarm controller
    swarm_controller = Node(
        package="multi_drone_pkg",
        executable="swarm_controller",
        name="swarm_controller",
        output="screen",
        parameters=[swarm_params],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=(
            [
                "-d",
                os.path.join(
                    get_package_share_directory("multi_drone_pkg"), "rviz", "swarm.rviz"
                ),
            ]
            if os.path.exists(
                os.path.join(
                    get_package_share_directory("multi_drone_pkg"), "rviz", "swarm.rviz"
                )
            )
            else []
        ),
    )

    return LaunchDescription(
        [
            *drone_nodes,
            swarm_controller,
            rviz_node,
        ]
    )
