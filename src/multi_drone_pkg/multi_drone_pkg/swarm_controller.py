#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    IntegerRange,
    SetParametersResult,
)
from geometry_msgs.msg import PoseStamped, Twist
import math
from collections import defaultdict
from enum import Enum, auto
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional


class SwarmState(Enum):
    """Explicit state machine states"""

    IDLE = auto()  # No neighbors, waiting
    FLOCK = auto()  # Normal flocking behavior
    FOLLOW = auto()  # Following leader
    AVOID = auto()  # Emergency collision avoidance
    SAFE = auto()  # Failure mode - minimal movement


@dataclass
class DroneInfo:
    """State estimation data for a single drone"""

    drone_id: int
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    last_seen: float
    is_active: bool
    distance_to_me: float = 0.0


@dataclass
class SwarmEstimate:
    """Complete state estimation for the swarm"""

    my_id: int
    active_drones: List[DroneInfo]
    neighbors: List[DroneInfo]
    leader_present: bool
    leader_info: Optional[DroneInfo]
    closest_distance: float
    swarm_center: Tuple[float, float, float]
    avg_velocity: Tuple[float, float, float]


class SwarmController(Node):
    """
    Phase 2: Structured Swarm Controller

    Separates:
    1. State Estimation - who/where/when
    2. Decision Logic - which state to be in
    3. Motion Generation - compute velocities
    4. Actuation - clamp and publish
    """

    def __init__(self):
        super().__init__("swarm_controller")

        # Callback group for parameter service responsiveness
        self.timer_callback_group = ReentrantCallbackGroup()

        # === SAFETY PARAMETERS ===
        self.declare_parameter(
            "collision_threshold",
            0.5,
            ParameterDescriptor(
                description="[SAFETY] Emergency collision distance (m)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "collision_weight",
            10.0,
            ParameterDescriptor(
                description="[SAFETY] Collision avoidance force multiplier",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=50.0, step=1.0)
                ],
            ),
        )

        self.declare_parameter(
            "timeout_seconds",
            1.0,
            ParameterDescriptor(
                description="[SAFETY] Drone timeout for lost detection (s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "leader_lost_timeout",
            2.0,
            ParameterDescriptor(
                description="[SAFETY] Leader lost timeout before state change (s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.5, to_value=10.0, step=0.5)
                ],
            ),
        )

        self.declare_parameter(
            "min_active_drones",
            2,
            ParameterDescriptor(
                description="[SAFETY] Minimum drones before SAFE mode",
                integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            ),
        )

        # === BEHAVIOR PARAMETERS ===
        self.declare_parameter(
            "separation_weight",
            1.5,
            ParameterDescriptor(
                description="[BEHAVIOR] Separation force weight",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "cohesion_weight",
            1.0,
            ParameterDescriptor(
                description="[BEHAVIOR] Cohesion force weight",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "alignment_weight",
            1.0,
            ParameterDescriptor(
                description="[BEHAVIOR] Alignment force weight",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "separation_distance",
            2.0,
            ParameterDescriptor(
                description="[BEHAVIOR] Separation activation distance (m)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.5, to_value=20.0, step=0.5)
                ],
            ),
        )

        self.declare_parameter(
            "neighbor_distance",
            10.0,
            ParameterDescriptor(
                description="[BEHAVIOR] Neighbor detection distance (m)",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=50.0, step=1.0)
                ],
            ),
        )

        self.declare_parameter(
            "bias_velocity_x",
            0.5,
            ParameterDescriptor(
                description="[BEHAVIOR] Bias velocity X (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=-5.0, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "bias_velocity_y",
            0.3,
            ParameterDescriptor(
                description="[BEHAVIOR] Bias velocity Y (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=-5.0, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "bias_velocity_z",
            0.0,
            ParameterDescriptor(
                description="[BEHAVIOR] Bias velocity Z (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.1)
                ],
            ),
        )

        # === LIMIT PARAMETERS ===
        self.declare_parameter(
            "max_speed",
            2.0,
            ParameterDescriptor(
                description="[LIMITS] Maximum drone speed (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_acceleration",
            1.0,
            ParameterDescriptor(
                description="[LIMITS] Maximum acceleration (m/s²)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "safe_mode_speed",
            0.5,
            ParameterDescriptor(
                description="[LIMITS] Speed in SAFE mode (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)
                ],
            ),
        )

        # === CONFIGURATION PARAMETERS ===
        self.declare_parameter(
            "num_drones",
            5,
            ParameterDescriptor(
                description="[CONFIG] Total number of drones",
                integer_range=[IntegerRange(from_value=1, to_value=20, step=1)],
            ),
        )

        self.declare_parameter(
            "leader_id",
            1,
            ParameterDescriptor(
                description="[CONFIG] Leader drone ID",
                integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            ),
        )

        self.declare_parameter(
            "leader_speed",
            0.5,
            ParameterDescriptor(
                description="[CONFIG] Leader circular speed (m/s)",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "leader_radius",
            3.0,
            ParameterDescriptor(
                description="[CONFIG] Leader circular radius (m)",
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=20.0, step=0.5)
                ],
            ),
        )

        # Parameter callback with validation and logging
        self.add_on_set_parameters_callback(self.parameter_callback_with_validation)

        # Get initial parameters
        self.num_drones = self.get_parameter("num_drones").value
        self.leader_id = self.get_parameter("leader_id").value

        # State machine
        self.current_state = SwarmState.IDLE
        self.state_entry_time = self.get_clock().now()
        self.leader_last_seen = None

        # Storage for drone states
        self.drone_poses = {}  # {drone_id: Pose}
        self.drone_velocities = defaultdict(lambda: [1.0, 0.5, 0.0])
        self.last_update_time = {}  # {drone_id: timestamp}

        # Leader state
        self.leader_angle = 0.0

        # Create subscribers and publishers
        self.pose_subscribers = {}
        self.vel_publishers = {}

        for drone_id in range(1, self.num_drones + 1):
            # Pose subscribers
            topic = f"/drone_{drone_id}/pose"
            self.pose_subscribers[drone_id] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, did=drone_id: self.pose_callback(msg, did),
                10,
            )

            # Velocity publishers
            topic = f"/drone_{drone_id}/cmd_vel"
            self.vel_publishers[drone_id] = self.create_publisher(Twist, topic, 10)

        # Control loop timer
        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.timer_callback_group
        )

        # Status reporting
        self.status_timer = self.create_timer(
            1.0, self.report_status, callback_group=self.timer_callback_group
        )

        self.get_logger().info(
            f"Swarm controller initialized (Phase 2): "
            f"num_drones={self.num_drones}, leader={self.leader_id}, state={self.current_state.name}"
        )

    # ===== PARAMETER VALIDATION =====

    def parameter_callback_with_validation(self, params):
        """Validate parameters and log changes as events"""
        result = SetParametersResult(successful=True)

        for param in params:
            # Validate based on parameter type
            if (
                "distance" in param.name
                or "threshold" in param.name
                or "radius" in param.name
            ):
                if param.value < 0:
                    self.get_logger().error(
                        f"❌ Parameter {param.name}={param.value} rejected (must be >= 0)"
                    )
                    result.successful = False
                    result.reason = f"{param.name} must be non-negative"
                    return result

            if "weight" in param.name:
                if param.value < 0:
                    self.get_logger().error(
                        f"❌ Parameter {param.name}={param.value} rejected (must be >= 0)"
                    )
                    result.successful = False
                    result.reason = f"{param.name} must be non-negative"
                    return result

            # Log parameter change as event
            self.get_logger().info(
                f"✅ Parameter changed: {param.name} = {param.value}"
            )

        return result

    # ===== SENSOR CALLBACKS =====

    def pose_callback(self, msg: PoseStamped, drone_id: int):
        """Store latest pose and update timestamp"""
        self.drone_poses[drone_id] = msg.pose
        self.last_update_time[drone_id] = self.get_clock().now()

        # Track leader
        if drone_id == self.leader_id:
            self.leader_last_seen = self.get_clock().now()

    # ===== 1. STATE ESTIMATION =====

    def estimate_swarm_state(self, my_drone_id: int) -> SwarmEstimate:
        """
        State Estimation: Extract all observable facts about the swarm
        - Who is active/alive?
        - Where are they?
        - Distances, velocities
        - Leader presence
        """
        current_time = self.get_clock().now()
        timeout = self.get_parameter("timeout_seconds").value
        neighbor_dist = self.get_parameter("neighbor_distance").value

        # Collect active drones
        active_drones = []
        for drone_id in range(1, self.num_drones + 1):
            if drone_id not in self.last_update_time:
                continue

            time_diff = (
                current_time - self.last_update_time[drone_id]
            ).nanoseconds / 1e9
            is_active = time_diff < timeout

            if is_active and drone_id in self.drone_poses:
                pose = self.drone_poses[drone_id]
                pos = (pose.position.x, pose.position.y, pose.position.z)
                vel = self.drone_velocities.get(drone_id, [0.0, 0.0, 0.0])

                info = DroneInfo(
                    drone_id=drone_id,
                    position=pos,
                    velocity=tuple(vel),
                    last_seen=time_diff,
                    is_active=True,
                )
                active_drones.append(info)

        # Calculate distances from my position
        if my_drone_id not in self.drone_poses:
            # If I don't have my own pose yet, return empty state
            return SwarmEstimate(
                my_id=my_drone_id,
                active_drones=[],
                neighbors=[],
                leader_present=False,
                leader_info=None,
                closest_distance=float("inf"),
                swarm_center=(0.0, 0.0, 1.0),
                avg_velocity=(0.0, 0.0, 0.0),
            )

        my_pose = self.drone_poses[my_drone_id]
        my_pos = (my_pose.position.x, my_pose.position.y, my_pose.position.z)

        # Calculate distances and identify neighbors
        neighbors = []
        closest_distance = float("inf")
        leader_info = None

        for drone_info in active_drones:
            if drone_info.drone_id == my_drone_id:
                continue

            # Calculate distance
            dx = drone_info.position[0] - my_pos[0]
            dy = drone_info.position[1] - my_pos[1]
            dz = drone_info.position[2] - my_pos[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            drone_info.distance_to_me = distance
            closest_distance = min(closest_distance, distance)

            # Check if neighbor
            if distance < neighbor_dist:
                neighbors.append(drone_info)

            # Check if leader
            if drone_info.drone_id == self.leader_id:
                leader_info = drone_info

        # Calculate swarm center
        if active_drones:
            cx = sum(d.position[0] for d in active_drones) / len(active_drones)
            cy = sum(d.position[1] for d in active_drones) / len(active_drones)
            cz = sum(d.position[2] for d in active_drones) / len(active_drones)
            swarm_center = (cx, cy, cz)

            # Average velocity
            vx = sum(d.velocity[0] for d in active_drones) / len(active_drones)
            vy = sum(d.velocity[1] for d in active_drones) / len(active_drones)
            vz = sum(d.velocity[2] for d in active_drones) / len(active_drones)
            avg_velocity = (vx, vy, vz)
        else:
            swarm_center = my_pos
            avg_velocity = (0.0, 0.0, 0.0)

        return SwarmEstimate(
            my_id=my_drone_id,
            active_drones=active_drones,
            neighbors=neighbors,
            leader_present=(leader_info is not None),
            leader_info=leader_info,
            closest_distance=closest_distance,
            swarm_center=swarm_center,
            avg_velocity=avg_velocity,
        )

    # ===== 2. DECISION LOGIC =====

    def decide_state(self, estimate: SwarmEstimate) -> SwarmState:
        """
        Decision Logic: Based on state estimation, decide which state to be in

        Transition rules:
        - ANY → AVOID if collision imminent
        - ANY → SAFE if too few drones
        - FOLLOW → FLOCK if leader lost
        - FLOCK → IDLE if no neighbors
        - IDLE → FLOCK if neighbors appear
        """
        collision_threshold = self.get_parameter("collision_threshold").value
        min_active = self.get_parameter("min_active_drones").value
        leader_timeout = self.get_parameter("leader_lost_timeout").value

        # Rule 1: AVOID if collision imminent (highest priority)
        # Stay in AVOID only if distance is critically small
        # Exit AVOID once drones have separated to safe distance
        if estimate.closest_distance < collision_threshold:
            if self.current_state != SwarmState.AVOID:
                self.get_logger().warn(
                    f"⚠️  STATE CHANGE: {self.current_state.name} → AVOID "
                    f"(closest={estimate.closest_distance:.2f}m < {collision_threshold}m)"
                )
                self.state_entry_time = self.get_clock().now()
            return SwarmState.AVOID

        # If we were in AVOID but distance is now safe, exit to FOLLOW/FLOCK
        if self.current_state == SwarmState.AVOID:
            if estimate.closest_distance >= collision_threshold * 1.5:  # Hysteresis
                self.get_logger().info(
                    f"ℹ️  STATE CHANGE: AVOID → FOLLOW (collision cleared, distance={estimate.closest_distance:.2f}m)"
                )
                self.state_entry_time = self.get_clock().now()
                return SwarmState.FOLLOW

        # Rule 2: SAFE if too few active drones
        if len(estimate.active_drones) < min_active:
            if self.current_state != SwarmState.SAFE:
                self.get_logger().error(
                    f"🛑 STATE CHANGE: {self.current_state.name} → SAFE "
                    f"(only {len(estimate.active_drones)} drones active, need {min_active})"
                )
                self.state_entry_time = self.get_clock().now()
            return SwarmState.SAFE

        # Rule 3: FOLLOW → FLOCK if leader lost
        if self.current_state == SwarmState.FOLLOW:
            if not estimate.leader_present:
                # Check how long leader has been missing
                if self.leader_last_seen is not None:
                    time_since_leader = (
                        self.get_clock().now() - self.leader_last_seen
                    ).nanoseconds / 1e9
                    if time_since_leader > leader_timeout:
                        self.get_logger().warn(
                            f"⚠️  STATE CHANGE: FOLLOW → FLOCK "
                            f"(leader lost for {time_since_leader:.1f}s > {leader_timeout}s)"
                        )
                        self.state_entry_time = self.get_clock().now()
                        return SwarmState.FLOCK
            return SwarmState.FOLLOW

        # Rule 4: FLOCK → IDLE if no neighbors
        if self.current_state == SwarmState.FLOCK:
            if len(estimate.neighbors) == 0:
                self.get_logger().info(f"ℹ️  STATE CHANGE: FLOCK → IDLE (no neighbors)")
                self.state_entry_time = self.get_clock().now()
                return SwarmState.IDLE
            # If we have neighbors, check if we should go to FOLLOW (Rule 6)
            # Transition to FOLLOW to enable circular motion
            self.get_logger().info(
                f"ℹ️  STATE CHANGE: FLOCK → FOLLOW (activating leader behavior)"
            )
            self.state_entry_time = self.get_clock().now()
            return SwarmState.FOLLOW

        # Rule 5: IDLE → FLOCK if neighbors appear
        if self.current_state == SwarmState.IDLE:
            if len(estimate.neighbors) > 0:
                self.get_logger().info(
                    f"ℹ️  STATE CHANGE: IDLE → FLOCK ({len(estimate.neighbors)} neighbors found)"
                )
                self.state_entry_time = self.get_clock().now()
                return SwarmState.FLOCK
            return SwarmState.IDLE

        # Default: stay in current state
        return self.current_state

    # ===== 3. MOTION GENERATION =====

    def generate_motion(
        self, estimate: SwarmEstimate, state: SwarmState
    ) -> Tuple[float, float, float]:
        """
        Motion Generation: Based on state, compute desired velocity
        Pure function - no side effects
        """
        if state == SwarmState.IDLE:
            return self._generate_idle_motion()

        elif state == SwarmState.FLOCK:
            return self._generate_flock_motion(estimate)

        elif state == SwarmState.FOLLOW:
            return self._generate_follow_motion(estimate)

        elif state == SwarmState.AVOID:
            return self._generate_avoid_motion(estimate)

        elif state == SwarmState.SAFE:
            return self._generate_safe_motion(estimate)

        else:
            return (0.0, 0.0, 0.0)

    def _generate_idle_motion(self) -> Tuple[float, float, float]:
        """IDLE: Minimal hovering motion"""
        return (0.0, 0.0, 0.0)

    def _generate_flock_motion(
        self, estimate: SwarmEstimate
    ) -> Tuple[float, float, float]:
        """FLOCK: Standard boids algorithm"""
        if estimate.my_id not in self.drone_poses:
            return (0.0, 0.0, 0.0)

        # Compute boids forces
        separation = self._compute_separation(estimate)
        cohesion = self._compute_cohesion(estimate)
        alignment = self._compute_alignment(estimate)

        # Get weights
        sep_w = self.get_parameter("separation_weight").value
        coh_w = self.get_parameter("cohesion_weight").value
        ali_w = self.get_parameter("alignment_weight").value

        # Get bias velocity
        bias_x = self.get_parameter("bias_velocity_x").value
        bias_y = self.get_parameter("bias_velocity_y").value
        bias_z = self.get_parameter("bias_velocity_z").value

        # Combine forces
        vx = sep_w * separation[0] + coh_w * cohesion[0] + ali_w * alignment[0] + bias_x
        vy = sep_w * separation[1] + coh_w * cohesion[1] + ali_w * alignment[1] + bias_y
        vz = sep_w * separation[2] + coh_w * cohesion[2] + ali_w * alignment[2] + bias_z

        return (vx, vy, vz)

    def _generate_follow_motion(
        self, estimate: SwarmEstimate
    ) -> Tuple[float, float, float]:
        """FOLLOW: Leader follows path, others chase leader"""
        if estimate.my_id == self.leader_id:
            # Leader follows circular path
            leader_speed = self.get_parameter("leader_speed").value
            leader_radius = self.get_parameter("leader_radius").value

            self.leader_angle += leader_speed * 0.1 / leader_radius

            vx = leader_speed * math.cos(self.leader_angle)
            vy = leader_speed * math.sin(self.leader_angle)
            return (vx, vy, 0.0)

        else:
            # Followers: move toward leader + some separation
            if not estimate.leader_present:
                return (0.0, 0.0, 0.0)

            leader = estimate.leader_info
            my_pos = self.drone_poses[estimate.my_id].position

            # Direction to leader
            dx = leader.position[0] - my_pos.x
            dy = leader.position[1] - my_pos.y
            dz = leader.position[2] - my_pos.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance > 0.1:
                vx = dx / distance * 1.5
                vy = dy / distance * 1.5
                vz = dz / distance * 1.5
            else:
                vx, vy, vz = 0.0, 0.0, 0.0

            # Add separation from other followers
            separation = self._compute_separation(estimate)
            vx += separation[0] * 0.5
            vy += separation[1] * 0.5
            vz += separation[2] * 0.5

            return (vx, vy, vz)

    def _generate_avoid_motion(
        self, estimate: SwarmEstimate
    ) -> Tuple[float, float, float]:
        """AVOID: Emergency collision avoidance - pure repulsion"""
        separation = self._compute_separation(estimate)
        weight = self.get_parameter("collision_weight").value

        return (separation[0] * weight, separation[1] * weight, separation[2] * weight)

    def _generate_safe_motion(
        self, estimate: SwarmEstimate
    ) -> Tuple[float, float, float]:
        """SAFE: Minimal movement, try to maintain position"""
        # Slowly drift toward swarm center if it exists
        if len(estimate.active_drones) > 0 and estimate.my_id in self.drone_poses:
            my_pos = self.drone_poses[estimate.my_id].position
            dx = estimate.swarm_center[0] - my_pos.x
            dy = estimate.swarm_center[1] - my_pos.y
            dz = estimate.swarm_center[2] - my_pos.z

            safe_speed = self.get_parameter("safe_mode_speed").value
            return (dx * 0.1 * safe_speed, dy * 0.1 * safe_speed, dz * 0.1 * safe_speed)

        return (0.0, 0.0, 0.0)

    # Boids force computation helpers

    def _compute_separation(
        self, estimate: SwarmEstimate
    ) -> Tuple[float, float, float]:
        """Repulsion from nearby neighbors"""
        if not estimate.neighbors or estimate.my_id not in self.drone_poses:
            return (0.0, 0.0, 0.0)

        separation_distance = self.get_parameter("separation_distance").value
        my_pos = self.drone_poses[estimate.my_id].position

        steer_x, steer_y, steer_z = 0.0, 0.0, 0.0
        count = 0

        for neighbor in estimate.neighbors:
            distance = neighbor.distance_to_me

            if distance < separation_distance and distance > 0.01:
                # Repulsion vector
                dx = my_pos.x - neighbor.position[0]
                dy = my_pos.y - neighbor.position[1]
                dz = my_pos.z - neighbor.position[2]

                # Weight by inverse distance squared
                steer_x += dx / (distance * distance)
                steer_y += dy / (distance * distance)
                steer_z += dz / (distance * distance)
                count += 1

        if count > 0:
            return (steer_x / count, steer_y / count, steer_z / count)

        return (0.0, 0.0, 0.0)

    def _compute_cohesion(self, estimate: SwarmEstimate) -> Tuple[float, float, float]:
        """Attraction to swarm center"""
        if not estimate.neighbors or estimate.my_id not in self.drone_poses:
            return (0.0, 0.0, 0.0)

        # Center of neighbors only (not absolute swarm center)
        cx = sum(n.position[0] for n in estimate.neighbors) / len(estimate.neighbors)
        cy = sum(n.position[1] for n in estimate.neighbors) / len(estimate.neighbors)
        cz = sum(n.position[2] for n in estimate.neighbors) / len(estimate.neighbors)

        my_pos = self.drone_poses[estimate.my_id].position

        # Direction to center
        dx = cx - my_pos.x
        dy = cy - my_pos.y
        dz = cz - my_pos.z

        # Scale down cohesion to allow drift
        return (dx * 0.5, dy * 0.5, dz * 0.5)

    def _compute_alignment(self, estimate: SwarmEstimate) -> Tuple[float, float, float]:
        """Match average velocity of neighbors"""
        if not estimate.neighbors:
            return (0.0, 0.0, 0.0)

        # Average neighbor velocity
        vx = sum(n.velocity[0] for n in estimate.neighbors) / len(estimate.neighbors)
        vy = sum(n.velocity[1] for n in estimate.neighbors) / len(estimate.neighbors)
        vz = sum(n.velocity[2] for n in estimate.neighbors) / len(estimate.neighbors)

        return (vx, vy, vz)

    # ===== 4. ACTUATION =====

    def actuate(self, drone_id: int, desired_velocity: Tuple[float, float, float]):
        """
        Actuation: Clamp velocity and publish commands
        Enforces physical limits
        """
        vx, vy, vz = desired_velocity

        # Get limits
        max_speed = self.get_parameter("max_speed").value
        max_accel = self.get_parameter("max_acceleration").value
        dt = 0.1

        # Get current velocity
        current_vel = self.drone_velocities[drone_id]

        # Limit acceleration
        dvx = vx - current_vel[0]
        dvy = vy - current_vel[1]
        dvz = vz - current_vel[2]

        accel = math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz) / dt

        if accel > max_accel:
            scale = (
                max_accel * dt / math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz + 0.001)
            )
            dvx *= scale
            dvy *= scale
            dvz *= scale

        # Apply acceleration limits
        vx = current_vel[0] + dvx
        vy = current_vel[1] + dvy
        vz = current_vel[2] + dvz

        # Limit speed
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > max_speed:
            vx = vx / speed * max_speed
            vy = vy / speed * max_speed
            vz = vz / speed * max_speed

        # Update stored velocity
        self.drone_velocities[drone_id] = [vx, vy, vz]

        # Publish command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz

        self.vel_publishers[drone_id].publish(cmd)

    # ===== MAIN CONTROL LOOP =====

    def control_loop(self):
        """Main control loop: runs the 4-stage pipeline for each drone"""
        active_drone_ids = []

        for drone_id in range(1, self.num_drones + 1):
            if drone_id not in self.drone_poses:
                continue

            # Stage 1: State Estimation
            estimate = self.estimate_swarm_state(drone_id)

            if not estimate.active_drones:
                continue

            active_drone_ids.append(drone_id)

            # Stage 2: Decision Logic (only update state once)
            if drone_id == 1:  # Only decide state once per control cycle
                self.get_logger().info(
                    f"🔍 Drone 1 estimate: neighbors={len(estimate.neighbors)}, current_state={self.current_state.name}",
                    throttle_duration_sec=1.0,
                )
                self.current_state = self.decide_state(estimate)

            # Stage 3: Motion Generation
            desired_velocity = self.generate_motion(estimate, self.current_state)

            # Stage 4: Actuation
            self.actuate(drone_id, desired_velocity)

    # ===== STATUS REPORTING =====

    def report_status(self):
        """Report swarm status"""
        if not self.drone_poses:
            return

        active_count = len(
            [
                d
                for d in range(1, self.num_drones + 1)
                if d in self.last_update_time
                and (self.get_clock().now() - self.last_update_time[d]).nanoseconds
                / 1e9
                < self.get_parameter("timeout_seconds").value
            ]
        )

        time_in_state = (
            self.get_clock().now() - self.state_entry_time
        ).nanoseconds / 1e9

        self.get_logger().info(
            f"=== STATE: {self.current_state.name} ({time_in_state:.1f}s) | "
            f"Active: {active_count}/{self.num_drones} ==="
        )

        for drone_id in sorted(self.drone_poses.keys()):
            pose = self.drone_poses[drone_id]
            vel = self.drone_velocities.get(drone_id, [0, 0, 0])
            speed = math.sqrt(sum(v**2 for v in vel))

            role = "LEADER" if drone_id == self.leader_id else "DRONE"
            self.get_logger().info(
                f"{role} {drone_id}: pos=({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) "
                f"speed={speed:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()

    # Use MultiThreadedExecutor for parameter service responsiveness
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
