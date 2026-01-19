# Technical Documentation: Multi-Drone Swarm System

This document provides an in-depth explanation of the algorithms, mathematics, and implementation details of the multi-drone swarm system.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [State Machine](#state-machine)
3. [Boids Algorithm](#boids-algorithm)
4. [Collision Avoidance](#collision-avoidance)
5. [Leader-Follower Dynamics](#leader-follower-dynamics)
6. [Motion Control](#motion-control)
7. [Implementation Details](#implementation-details)
8. [Performance Analysis](#performance-analysis)

---

## System Architecture

### Overview

The system implements a **centralized control architecture** where a single swarm controller manages all drones. This contrasts with distributed approaches but offers several advantages:

- **Global state awareness**: Controller sees all drone positions
- **Simplified coordination**: No consensus protocols needed
- **Easier debugging**: Single point of logic
- **Deterministic behavior**: Predictable state transitions

### Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Swarm Controller Node                      │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐    │
│  │     State      │  │    Decision    │  │     Motion      │    │
│  │   Estimation   │→ │     Logic      │→ │   Generation    │    │
│  │                │  │  (FSM)         │  │   (Boids)       │    │
│  └────────────────┘  └────────────────┘  └─────────────────┘    │
│           ↑                                        ↓            │
│      pose_sub[1..N]                          vel_pub[1..N]      │
└───────────┬────────────────────────────────────────┬─────────── ┘
            │ /drone_i/pose                 /drone_i/cmd_vel │
            │ (PoseStamped)                      (Twist)     │
┌───────────┴────────────────────────────────────────┴───────────┐
│                       Drone Nodes (N)                          │
│  ┌─────────────┐  ┌─────────────┐       ┌─────────────┐        │
│  │  Drone 1    │  │  Drone 2    │  ...  │  Drone N    │        │
│  │ (Kinematic) │  │ (Kinematic) │       │ (Kinematic) │        │
│  └─────────────┘  └─────────────┘       └─────────────┘        │
└────────────────────────────────────────────────────────────────┘
```

### Data Flow

**Control Loop (10 Hz)**:
```
1. Read poses from all drones
2. For each drone i:
   a. Estimate state (neighbors, distances, etc.)
   b. Decide next state (FSM)
   c. Generate desired velocity (Boids/Follow/Avoid)
   d. Apply velocity limits
   e. Publish velocity command
3. Sleep until next iteration
```

**Drone Simulation (variable rate)**:
```
1. Receive velocity command
2. Integrate velocity → position (Euler integration)
3. Publish updated pose
```

---

## State Machine

### State Definitions

The swarm controller implements a **Finite State Machine (FSM)** with 5 states:

```python
class SwarmState(Enum):
    IDLE = 0      # No neighbors detected
    FLOCK = 1     # Basic flocking behavior
    FOLLOW = 2    # Leader-follower mode
    AVOID = 3     # Collision avoidance (highest priority)
    SAFE = 4      # Fail-safe mode (degraded operation)
```

### State Transition Diagram

```
           ┌──────────────────────────────────────┐
           │         collision_detected           │
           │                                      │
           v                                      │
        ┌──────┐                                  │
    ┌──>│ IDLE │<────no_neighbors─────┐           │
    │   └──────┘                      │           │
    │      │                          │           │
    │      │ neighbors_detected       │           │
    │      v                          │           │
    │   ┌──────┐    leader_present    ┌────────┐  │
    │   │FLOCK │─────────────────────>│ FOLLOW │<─┘
    │   └──────┘                      └────────┘
    │      │                              │  ^
    │      │                              │  │
    │      │                              v  │
    │      │                           ┌──────┐
    │      └─────too_few_drones───────>│ SAFE │
    │                                  └──────┘
    │                                      │
    │                                      │
    │                                      v
    │                                  ┌──────┐
    └──────────drones_recovered────────│AVOID │
                                       └──────┘
                                           │
                                           │
                                           └─>collision_cleared
```

### Transition Rules

**Priority Order** (highest to lowest):

1. **ANY → AVOID**: `if closest_distance < collision_threshold`
2. **ANY → SAFE**: `if active_drones < min_active_drones`
3. **FOLLOW → FLOCK**: `if !leader_present && time_since_leader > timeout`
4. **FLOCK → FOLLOW**: `if leader_present && neighbors > 0`
5. **FLOCK → IDLE**: `if neighbors == 0`
6. **IDLE → FLOCK**: `if neighbors > 0`
7. **AVOID → FOLLOW**: `if closest_distance >= collision_threshold × 1.5`

### Hysteresis in Collision Avoidance

To prevent oscillation between FOLLOW and AVOID states:

```python
# Enter AVOID
if distance < collision_threshold:
    state = AVOID

# Exit AVOID (hysteresis factor = 1.5)
if state == AVOID and distance >= collision_threshold * 1.5:
    state = FOLLOW
```

This creates a "dead zone" where:
- Enter AVOID at: distance < 0.3m
- Exit AVOID at: distance > 0.45m (1.5 × 0.3)
- Gap: 0.15m prevents rapid switching

---

## Boids Algorithm

### Original Formulation (Reynolds, 1987)

The Boids algorithm simulates flocking behavior using three simple rules:

1. **Separation**: Avoid crowding nearby flockmates
2. **Alignment**: Steer towards average heading of flockmates
3. **Cohesion**: Move towards average position of flockmates

### Mathematical Model

For drone $i$ at position $\mathbf{p}_i$ with velocity $\mathbf{v}_i$:

#### 1. Separation Force

$$\mathbf{F}_{\text{sep},i} = -\sum_{j \in N_i} \frac{\mathbf{p}_j - \mathbf{p}_i}{||\mathbf{p}_j - \mathbf{p}_i||^2}$$

Where:
- $N_i$ = set of neighbors within separation distance
- Force increases as $1/d^2$ (inverse square law)
- Direction: away from neighbors

**Implementation:**
```python
def _compute_separation(self, estimate):
    steer = Vector3(0, 0, 0)
    count = 0
    
    for neighbor in estimate.neighbors:
        distance = neighbor.distance_to_me
        if distance < separation_distance and distance > 0.01:
            # Repulsion vector
            diff = my_position - neighbor.position
            # Weight by inverse distance squared
            steer += diff / (distance * distance)
            count += 1
    
    if count > 0:
        steer /= count  # Average
    
    return steer * separation_weight
```

**Physical Interpretation:**
- Acts like electrostatic repulsion
- Stronger when drones are closer
- Prevents collisions in normal operation

#### 2. Cohesion Force

$$\mathbf{F}_{\text{coh},i} = \frac{1}{|N_i|}\sum_{j \in N_i} \mathbf{p}_j - \mathbf{p}_i$$

Where:
- $\frac{1}{|N_i|}\sum_{j \in N_i} \mathbf{p}_j$ = center of mass of neighbors
- Force direction: towards group center

**Implementation:**
```python
def _compute_cohesion(self, estimate):
    if not estimate.neighbors:
        return Vector3(0, 0, 0)
    
    # Calculate center of neighbors
    center = sum(n.position for n in neighbors) / len(neighbors)
    
    # Direction to center
    desired = center - my_position
    
    # Scale down cohesion (0.5x)
    return desired * 0.5 * cohesion_weight
```

**Physical Interpretation:**
- Acts like gravitational attraction
- Keeps swarm together
- Counterbalances separation

#### 3. Alignment Force

$$\mathbf{F}_{\text{ali},i} = \frac{1}{|N_i|}\sum_{j \in N_i} \mathbf{v}_j - \mathbf{v}_i$$

Where:
- $\frac{1}{|N_i|}\sum_{j \in N_i} \mathbf{v}_j$ = average velocity of neighbors
- Force direction: match group velocity

**Implementation:**
```python
def _compute_alignment(self, estimate):
    if not estimate.neighbors:
        return Vector3(0, 0, 0)
    
    # Average velocity of neighbors
    avg_velocity = sum(n.velocity for n in neighbors) / len(neighbors)
    
    # Desired velocity change
    desired = avg_velocity - my_velocity
    
    return desired * alignment_weight
```

**Physical Interpretation:**
- Acts like velocity damping
- Synchronizes motion
- Maintains coordinated movement

### Combined Flocking Force

$$\mathbf{F}_{\text{total},i} = w_s \mathbf{F}_{\text{sep},i} + w_c \mathbf{F}_{\text{coh},i} + w_a \mathbf{F}_{\text{ali},i} + \mathbf{v}_{\text{bias}}$$

Where:
- $w_s, w_c, w_a$ = tunable weights
- $\mathbf{v}_{\text{bias}}$ = external bias (e.g., wind, target direction)

**Default Weights:**
- $w_s = 1.5$ (separation: highest priority)
- $w_c = 1.0$ (cohesion: moderate)
- $w_a = 1.0$ (alignment: moderate)
- $\mathbf{v}_{\text{bias}} = (0, 0, 0)$ (no external force)

### Neighbor Detection

Neighbors are determined by:

```python
is_neighbor = (distance < neighbor_distance) and (time_since_update < timeout)
```

**Parameters:**
- `neighbor_distance`: 15.0m (detection radius)
- `timeout`: 1.0s (stale data threshold)

**Visibility Model:**
```
    Drone i
       ●
       │
       │ neighbor_distance = 15m
       │
   ════╪════════════════════════
       │             ●
       │          Drone j
   Inside:       (neighbor)
   
   
                             ●  Drone k
                          (not neighbor)
```

---

## Collision Avoidance

### AVOID State Behavior

In AVOID state, the swarm controller generates **pure repulsion forces** to maximize separation:

$$\mathbf{v}_{\text{avoid},i} = w_{\text{collision}} \cdot \mathbf{F}_{\text{sep},i}$$

Where:
- $w_{\text{collision}} = 10.0$ (much larger than normal separation weight)
- $\mathbf{F}_{\text{sep},i}$ uses same computation as Boids separation

**Key Differences from Normal Separation:**

| Aspect | Normal (FLOCK/FOLLOW) | Emergency (AVOID) |
|--------|----------------------|-------------------|
| Weight | 1.5 | 10.0 |
| Priority | Shared with cohesion/alignment | Exclusive |
| State duration | Continuous | Until collision cleared |

### Collision Detection

```python
closest_distance = min(distance_to_neighbor for all neighbors)

if closest_distance < collision_threshold:
    enter_AVOID_state()
```

**Collision Threshold:**
- Default: 0.3m
- Physical meaning: Minimum safe distance
- Consideration: Drone size + safety margin

### Energy Function Perspective

The system can be viewed as minimizing an energy function:

$$E = E_{\text{separation}} + E_{\text{cohesion}} + E_{\text{alignment}}$$

Where:

$$E_{\text{separation}} = \sum_{i,j} \frac{k_s}{||\mathbf{p}_i - \mathbf{p}_j||} \quad \text{(repulsive)}$$

$$E_{\text{cohesion}} = \sum_i ||\mathbf{p}_i - \mathbf{p}_{\text{center}}||^2 \quad \text{(attractive)}$$

$$E_{\text{alignment}} = \sum_i ||\mathbf{v}_i - \mathbf{v}_{\text{avg}}||^2 \quad \text{(damping)}$$

In AVOID mode, $k_s$ is dramatically increased, making $E_{\text{separation}}$ dominant.

---

## Leader-Follower Dynamics

### Leader Motion (Circular Path)

The leader (Drone 1) follows a circular trajectory:

$$\theta(t) = \theta_0 + \omega t$$

$$x(t) = x_c + R \cos(\theta(t))$$

$$y(t) = y_c + R \sin(\theta(t))$$

$$z(t) = z_0 \quad \text{(constant altitude)}$$

Where:
- $R$ = `leader_radius` (default: 5.0m)
- $\omega = \frac{v}{R}$ (angular velocity)
- $v$ = `leader_speed` (default: 1.0 m/s)
- $(x_c, y_c)$ = circle center (initially at drone's start position)

**Velocity Commands:**

$$v_x = v \cos(\theta) = v \cos(\omega t)$$

$$v_y = v \sin(\theta) = v \sin(\omega t)$$

$$v_z = 0$$

**Implementation:**
```python
def _generate_follow_motion_leader(self):
    # Update angle
    dt = 0.1  # Control loop period
    self.leader_angle += leader_speed * dt / leader_radius
    
    # Compute velocity
    vx = leader_speed * math.cos(self.leader_angle)
    vy = leader_speed * math.sin(self.leader_angle)
    vz = 0.0
    
    return (vx, vy, vz)
```

**Period of One Circle:**

$$T = \frac{2\pi R}{v} = \frac{2\pi \times 5.0}{1.0} = 31.4 \text{ seconds}$$

### Follower Motion

Followers use **modified flocking** with additional attraction to leader:

$$\mathbf{v}_{\text{follower},i} = \mathbf{F}_{\text{sep},i} + \mathbf{F}_{\text{coh},i} + \mathbf{F}_{\text{ali},i} + \mathbf{F}_{\text{leader},i}$$

Where:

$$\mathbf{F}_{\text{leader},i} = k_L (\mathbf{p}_{\text{leader}} - \mathbf{p}_i)$$

**Physical Meaning:**
- Followers maintain formation using Boids
- Additional force pulls them toward leader's position
- Leader's circular motion "drags" the formation

**Formation Stability:**

The formation is stable when:
1. Cohesion force balances separation
2. Leader attraction prevents drift
3. Alignment synchronizes motion

Mathematically, at equilibrium:

$$\sum \mathbf{F}_i = 0$$

But leader's motion prevents true equilibrium, resulting in dynamic steady state (circular motion).

---

## Motion Control

### Velocity Limits

To ensure safe and realistic motion, velocity commands are clamped:

#### 1. Acceleration Limit

$$\Delta \mathbf{v} = \mathbf{v}_{\text{desired}} - \mathbf{v}_{\text{current}}$$

$$a = \frac{||\Delta \mathbf{v}||}{dt}$$

If $a > a_{\max}$:

$$\Delta \mathbf{v} \leftarrow \Delta \mathbf{v} \cdot \frac{a_{\max} \cdot dt}{||\Delta \mathbf{v}||}$$

$$\mathbf{v}_{\text{new}} = \mathbf{v}_{\text{current}} + \Delta \mathbf{v}$$

**Parameters:**
- $a_{\max}$ = `max_acceleration` = 1.0 m/s²
- $dt$ = 0.1s (control loop period)

**Physical Meaning:**
- Prevents instantaneous velocity changes
- Models actuator bandwidth limitations
- Smoother, more realistic motion

#### 2. Speed Limit

$$v = ||\mathbf{v}_{\text{new}}||$$

If $v > v_{\max}$:

$$\mathbf{v}_{\text{new}} \leftarrow \mathbf{v}_{\text{new}} \cdot \frac{v_{\max}}{v}$$

**Parameters:**
- $v_{\max}$ = `max_speed` = 2.0 m/s

**Physical Meaning:**
- Prevents unrealistic velocities
- Models maximum thrust capability
- Ensures stability at high speeds

### Control Loop Timing

```python
while rclpy.ok():
    start_time = time.now()
    
    # Control logic (state estimation, motion generation, etc.)
    for drone in drones:
        process_drone(drone)
    
    # Sleep to maintain 10 Hz
    elapsed = time.now() - start_time
    sleep_time = (1.0 / 10.0) - elapsed
    if sleep_time > 0:
        sleep(sleep_time)
```

**Update Rate:**
- Control loop: 10 Hz (0.1s period)
- Status reporting: 1 Hz (1.0s period)

**Rationale:**
- 10 Hz sufficient for kinematic control
- Higher rates waste CPU
- Lower rates reduce responsiveness

---

## Implementation Details

### Data Structures

#### SwarmEstimate

```python
@dataclass
class SwarmEstimate:
    my_id: int                          # This drone's ID
    active_drones: List[DroneInfo]      # All active drones
    neighbors: List[DroneInfo]          # Nearby drones
    leader_present: bool                # Is leader visible?
    leader_info: Optional[DroneInfo]    # Leader's state
    closest_distance: float             # Distance to nearest neighbor
    swarm_center: Tuple[float, float, float]  # COM of swarm
    avg_velocity: Tuple[float, float, float]  # Average velocity
```

#### DroneInfo

```python
@dataclass
class DroneInfo:
    drone_id: int                       # Unique identifier
    position: Tuple[float, float, float]  # (x, y, z) in meters
    velocity: Tuple[float, float, float]  # (vx, vy, vz) in m/s
    last_seen: float                    # Time since last update
    is_active: bool                     # Currently active?
    distance_to_me: float               # Distance to observer
```

### Coordinate Frames

**World Frame** (fixed):
- Origin: (0, 0, 0)
- X-axis: East
- Y-axis: North
- Z-axis: Up

**Drone Frame** (body-fixed):
- Not used in this implementation (kinematic model)
- All calculations in world frame

### Message Types

**Pose Topic** (`/drone_X/pose`):
```
geometry_msgs/PoseStamped
├── header
│   ├── stamp
│   └── frame_id: "world"
└── pose
    ├── position
    │   ├── x: float
    │   ├── y: float
    │   └── z: float
    └── orientation (not used)
```

**Velocity Topic** (`/drone_X/cmd_vel`):
```
geometry_msgs/Twist
├── linear
│   ├── x: float (forward velocity)
│   ├── y: float (lateral velocity)
│   └── z: float (vertical velocity)
└── angular (not used)
```

### Kinematic Simulation

Drone nodes implement simple Euler integration:

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t) \cdot \Delta t$$

**Implementation:**
```python
def cmd_vel_callback(self, msg):
    dt = 0.1  # Approximate time step
    
    # Update position
    self.position.x += msg.linear.x * dt
    self.position.y += msg.linear.y * dt
    self.position.z += msg.linear.z * dt
    
    # Publish new pose
    self.publish_pose()
```

**Limitations:**
- No dynamics (mass, inertia)
- No external forces (gravity, drag)
- No attitude control
- Instantaneous velocity response (unrealistic)

**Advantages:**
- Computationally cheap
- Deterministic
- Good for algorithm testing
- No physics engine dependency

---

## Performance Analysis

### Computational Complexity

**Per Control Cycle:**

| Operation | Complexity | Count | Total |
|-----------|-----------|-------|-------|
| State estimation | O(N) | N drones | O(N²) |
| Neighbor search | O(N) per drone | N drones | O(N²) |
| Boids computation | O(K) per drone, K=neighbors | N drones | O(NK) |
| Velocity limiting | O(1) | N drones | O(N) |

**Overall:** O(N²) where N = number of drones

**Practical Performance:**
- 5 drones: ~5-10% CPU (Intel i5-8250U @ 1.6GHz)
- 10 drones: ~15-25% CPU (estimated)
- 50 drones: ~60-80% CPU (estimated)

### Scalability

**Centralized Architecture Limits:**
1. **Communication Overhead**: All poses → controller → all velocities
2. **Single Point of Failure**: Controller crash = swarm failure
3. **Computation Bottleneck**: O(N²) doesn't scale to large swarms

**Distributed Alternative (Not Implemented):**
- Each drone runs its own controller
- Communication: only with neighbors
- Complexity: O(K) where K << N
- Scales to hundreds of drones

### Memory Usage

**Per Drone:**
- Pose storage: ~100 bytes
- Velocity storage: ~50 bytes
- Metadata: ~50 bytes
- Total: ~200 bytes/drone

**For 5 Drones:**
- Total RAM: ~50 MB (mostly Python overhead)
- Actual data: ~1 KB

---

## Mathematical Proofs and Properties

### Stability Analysis

**Lyapunov Function Approach:**

Define energy function:

$$V = \sum_{i < j} \frac{1}{2} k_s \frac{1}{||\mathbf{p}_i - \mathbf{p}_j||} + \sum_i \frac{1}{2} k_c ||\mathbf{p}_i - \mathbf{p}_c||^2$$

Where $\mathbf{p}_c$ is swarm center.

**Theorem:** If separation dominates ($k_s >> k_c$), the system is collision-free.

**Proof sketch:**
1. As $||\mathbf{p}_i - \mathbf{p}_j|| \to 0$, $V \to \infty$
2. System minimizes $V$
3. Therefore, $||\mathbf{p}_i - \mathbf{p}_j||$ bounded away from 0

**Practical Limitation:**
- Discrete time integration
- Velocity limits
- External disturbances

### Formation Geometry

**Question:** What formation do drones naturally adopt in FLOCK mode?

**Answer:** Approximately hexagonal close-packing (2D).

**Reasoning:**
1. Separation creates repulsion
2. Cohesion creates attraction
3. Equilibrium when forces balance
4. Minimum energy configuration is hexagonal lattice

**Observed Behavior:**
- Small swarms (N < 10): Irregular but spread out
- Large swarms (N > 50): Hexagonal-like patterns emerge

---

## Advanced Topics

### Extension: 3D Flocking

Current implementation uses 2D motion (constant altitude). To extend to 3D:

1. **Remove altitude constraint** in leader motion:
```python
# Current: vz = 0.0
# 3D version:
vz = leader_speed * math.sin(elevation_angle)
```

2. **Add vertical separation** in Boids forces

3. **Consider 3D obstacle avoidance**

### Extension: Dynamic Leader Election

Current implementation has fixed leader (Drone 1). Dynamic election:

1. **Criteria for leader**:
   - Most central drone
   - Highest battery level
   - Longest time airborne

2. **Election algorithm**:
   - Periodic voting
   - Leader broadcasts heartbeat
   - Followers detect leader loss → elect new

### Extension: Obstacle Avoidance

Add static obstacles to environment:

1. **Obstacle representation**:
```python
obstacles = [
    {'position': (5.0, 5.0, 1.0), 'radius': 2.0},
    {'position': (10.0, -5.0, 1.0), 'radius': 1.5},
]
```

2. **Repulsion force**:
$$\mathbf{F}_{\text{obstacle}} = -k_o \sum_{\text{obstacles}} \frac{\mathbf{p}_{\text{obs}} - \mathbf{p}_i}{||\mathbf{p}_{\text{obs}} - \mathbf{p}_i||^2}$$

3. **Add to total force**:
$$\mathbf{F}_{\text{total}} += w_o \mathbf{F}_{\text{obstacle}}$$

---

## References

### Key Papers

1. **Reynolds, C. W.** (1987). *Flocks, herds and schools: A distributed behavioral model.* ACM SIGGRAPH Computer Graphics, 21(4), 25-34.
   - Original Boids paper
   - Foundational work on flocking

2. **Olfati-Saber, R.** (2006). *Flocking for multi-agent dynamic systems: Algorithms and theory.* IEEE Transactions on Automatic Control, 51(3), 401-420.
   - Rigorous mathematical framework
   - Stability proofs

3. **Vásárhelyi, G., et al.** (2018). *Optimized flocking of autonomous drones in confined environments.* Science Robotics, 3(20).
   - Real-world drone experiments
   - Collision avoidance strategies

4. **Tanner, H. G., et al.** (2007). *Flocking in fixed and switching networks.* IEEE Transactions on Automatic Control, 52(5), 863-868.
   - Formation control theory
   - Switching topologies

### Books

1. **Bullo, F., Cortés, J., & Martínez, S.** (2009). *Distributed Control of Robotic Networks.* Princeton University Press.
   - Comprehensive coverage of multi-agent systems

2. **Mesbahi, M., & Egerstedt, M.** (2010). *Graph Theoretic Methods in Multiagent Networks.* Princeton University Press.
   - Graph theory approach to coordination

### Online Resources

- ROS2 Documentation: https://docs.ros.org/en/humble/
- Craig Reynolds' Boids: https://www.red3d.com/cwr/boids/
- Swarm Robotics Survey: https://arxiv.org/abs/2104.09224

---

## Glossary

**Boids**: Artificial life simulation of flocking behavior (bird-oid objects)

**Cohesion**: Tendency of agents to move toward the center of their local neighborhood

**Separation**: Tendency of agents to avoid crowding their neighbors

**Alignment**: Tendency of agents to match the velocity of their neighbors

**Leader-Follower**: Control architecture where one agent (leader) dictates motion, others follow

**Kinematic Model**: Motion model considering only position and velocity (not forces/accelerations)

**Hysteresis**: Dependence of system state on history, used to prevent oscillation

**Finite State Machine (FSM)**: Computational model with discrete states and defined transitions

**Swarm Intelligence**: Collective behavior of decentralized, self-organized systems

---

**Document Version**: 1.0  
**Last Updated**: January 19, 2026  
**Author**: Nafees  
**For Questions**: See README.md or open an issue on GitHub
