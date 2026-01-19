# Getting Started with Multi-Drone Swarm System

This guide will walk you through setting up and running your first drone swarm simulation.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Your First Swarm](#your-first-swarm)
4. [Understanding the Simulation](#understanding-the-simulation)
5. [Customizing Parameters](#customizing-parameters)
6. [Common Issues](#common-issues)
7. [Next Steps](#next-steps)

---

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 (Jammy Jellyfish)
- **RAM**: Minimum 4GB (8GB recommended)
- **CPU**: Modern multi-core processor
- **GPU**: Not required (pure kinematic simulation)

### Software Requirements

1. **ROS2 Humble**: Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html)

```bash
# Quick install (if not already installed)
sudo apt update && sudo apt install -y ros-humble-desktop
```

2. **Python 3.10+**: Should be included with Ubuntu 22.04

```bash
python3 --version  # Should show 3.10.x
```

3. **Additional ROS2 Packages**:

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rviz2 \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs
```

---

## Installation

### Step 1: Create Workspace

```bash
# Create a new ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone the Repository

```bash
# Clone the package 
git clone https://github.com/codemaster1000/multi-drone-coordination multi_drone_pkg

# Or if you have the source code locally
cp -r /path/to/multi_drone_pkg .
```

### Step 3: Install Dependencies

```bash
cd ~/ros2_ws

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Package

```bash
# Build the workspace
colcon build --packages-select multi_drone_pkg

# This should complete without errors
# You'll see: "Summary: 1 package finished"
```

### Step 5: Source the Workspace

```bash
# Add to current terminal
source ~/ros2_ws/install/setup.bash

# Add to .bashrc for automatic sourcing (optional but recommended)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Verify Installation

```bash
# Check if the package is visible
ros2 pkg list | grep multi_drone_pkg

# You should see: multi_drone_pkg
```

---

## Your First Swarm

### Launch the Simulation

Open a terminal and run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch multi_drone_pkg rviz_swarm_launch.py
```

You should see:
1. **Terminal Output**: Status messages showing drones initializing
2. **RViz Window**: 3D visualization with 5 colored drones

### What You'll See

Initially:
- 5 drones arranged in a **pentagon formation**
- Drones in **IDLE** state (not moving)

After ~1 second:
- State changes to **FLOCK** (drones detect neighbors)
- State changes to **FOLLOW** (circular motion activated)
- **Drone 1 (blue)** starts flying in a circle
- Other drones follow while maintaining formation

### Understanding the Display

**RViz Elements:**
- **Grid**: World coordinate frame
- **Colored axes**: Each drone's orientation
- **Drone trails**: Movement history (if enabled)

**Terminal Output:**
```
[INFO] [swarm_controller]: === STATE: FOLLOW (5.2s) | Active: 5/5 ===
[INFO] [swarm_controller]: LEADER 1: pos=(2.34, 4.12, 1.00) speed=1.00 m/s
[INFO] [swarm_controller]: DRONE 2: pos=(3.45, 3.21, 1.00) speed=0.95 m/s
...
```

**State Messages:**
- `STATE CHANGE: IDLE → FLOCK`: Neighbors detected
- `STATE CHANGE: FLOCK → FOLLOW`: Leader behavior activated
- `STATE CHANGE: FOLLOW → AVOID`: Collision detected
- `STATE CHANGE: AVOID → FOLLOW`: Collision cleared

### Stop the Simulation

Press `Ctrl+C` in the terminal where you launched the system.

---

## Understanding the Simulation

### System Components

#### 1. Drone Nodes (5 instances)

Each drone is an independent ROS2 node:
- **Subscribes to**: `/drone_X/cmd_vel` (velocity commands)
- **Publishes to**: `/drone_X/pose` (current position)
- **Behavior**: Kinematic simulation (integrates velocity to update position)

#### 2. Swarm Controller (1 instance)

The centralized "brain":
- **Subscribes to**: All drone poses
- **Publishes to**: All drone velocity commands
- **Behavior**: Implements swarm intelligence algorithms

#### 3. RViz2

Visualization tool:
- Displays drone positions in real-time
- Shows grid and reference frames
- Configurable via `rviz/swarm.rviz`

### State Machine Overview

```
IDLE ──neighbor_detected──> FLOCK ──leader_present──> FOLLOW
  ^                            |                          |
  |                            |                          |
  └──no_neighbors──────────────┘                          |
                                                          |
        ┌─────────────────────────────────────────────────┘
        |
        v
    collision_detected
        |
        v
     AVOID ──collision_cleared──> FOLLOW
```

**States Explained:**

1. **IDLE**: Waiting for neighbors
   - Action: Hover in place
   - Exit: When neighbors appear within detection radius

2. **FLOCK**: Boids-based flocking
   - Action: Separation + Cohesion + Alignment
   - Exit: Immediately transitions to FOLLOW

3. **FOLLOW**: Circular formation
   - Leader: Flies in a circle (radius = 5m, speed = 1 m/s)
   - Followers: Maintain formation using flocking
   - Exit: When collision detected

4. **AVOID**: Emergency collision avoidance
   - Action: Strong repulsion from nearby drones
   - Exit: When distance > 0.45m (1.5 × collision_threshold)

5. **SAFE**: Fail-safe mode (rarely entered)
   - Action: Minimal movement
   - Exit: When enough drones become active

### Boids Algorithm

The swarm uses three fundamental behaviors:

**1. Separation** (Personal Space)
```
Keep away from nearby drones
Force ∝ 1/distance²
```

**2. Cohesion** (Stay Together)
```
Move toward the center of nearby drones
Force ∝ (center_position - my_position)
```

**3. Alignment** (Match Speed)
```
Match the average velocity of nearby drones
Force ∝ (average_velocity - my_velocity)
```

---

## Customizing Parameters

### Method 1: Edit Launch File (Permanent)

Edit `launch/rviz_swarm_launch.py`:

```python
swarm_params = {
    "leader_speed": 2.0,      # Change from 1.0 to 2.0
    "leader_radius": 8.0,     # Change from 5.0 to 8.0
    "separation_weight": 2.0, # Change from 1.5 to 2.0
}
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select multi_drone_pkg
source install/setup.bash
ros2 launch multi_drone_pkg rviz_swarm_launch.py
```

### Method 2: Command-Line Override (Temporary)

```bash
ros2 launch multi_drone_pkg rviz_swarm_launch.py \
  leader_speed:=2.0 \
  leader_radius:=8.0 \
  separation_weight:=2.0
```

### Common Parameter Tweaks

**Faster Circular Motion:**
```bash
leader_speed:=3.0 leader_radius:=10.0
```

**Tighter Formation:**
```bash
cohesion_weight:=2.0 separation_weight:=1.0
```

**More Aggressive Separation:**
```bash
separation_weight:=3.0 collision_threshold:=0.5
```

**Add Drift (Wind Effect):**
```bash
bias_velocity_x:=0.5 bias_velocity_y:=0.3
```

---

## Common Issues

### Issue 1: "Package not found"

**Error:**
```
Package 'multi_drone_pkg' not found
```

**Solution:**
```bash
cd ~/ros2_ws
colcon build --packages-select multi_drone_pkg
source install/setup.bash
```

### Issue 2: RViz doesn't show drones

**Problem**: RViz opens but grid is empty

**Solution:**
1. Check if nodes are running:
```bash
ros2 node list
# Should show: /drone_1/drone_node, /drone_2/drone_node, ..., /swarm_controller
```

2. Check topics:
```bash
ros2 topic list
# Should show: /drone_1/pose, /drone_2/pose, ...
```

3. Check RViz configuration:
   - In RViz, check that `Fixed Frame` is set to `world`
   - Verify PoseStamped displays are enabled

### Issue 3: Drones stuck in AVOID state

**Problem**: Terminal shows continuous AVOID messages, drones not moving

**Symptom:**
```
[WARN] STATE CHANGE: FOLLOW → AVOID (closest=0.29m < 0.3m)
[INFO] STATE CHANGE: AVOID → FOLLOW (collision cleared, distance=0.52m)
[WARN] STATE CHANGE: FOLLOW → AVOID (closest=0.28m < 0.3m)
```

**Solution**: Increase collision threshold
```bash
# In launch file:
"collision_threshold": 0.5,  # Change from 0.3
```

### Issue 4: Drones drift in straight line

**Problem**: Swarm moves in one direction instead of circling

**Cause**: Bias velocity is non-zero

**Solution**:
```bash
# In launch file, ensure:
"bias_velocity_x": 0.0,
"bias_velocity_y": 0.0,
"bias_velocity_z": 0.0,
```

### Issue 5: Build errors

**Error:**
```
--- stderr: multi_drone_pkg
ModuleNotFoundError: No module named 'setuptools'
```

**Solution:**
```bash
sudo apt install python3-setuptools
cd ~/ros2_ws
colcon build --packages-select multi_drone_pkg
```

---

## Next Steps

### Experiment with Different Configurations

1. **Change Number of Drones**:
   - Edit `rviz_swarm_launch.py`
   - Add more drone nodes
   - Update `num_drones` parameter

2. **Create Custom Formations**:
   - Modify starting positions in launch file
   - Try: line, circle, grid, random

3. **Tune Swarm Behavior**:
   - Adjust weights to see different flocking patterns
   - Try extreme values to understand limits

### Advanced Topics

1. **Run Without RViz** (lightweight):
```bash
ros2 launch multi_drone_pkg swarm_launch.py
```

2. **Monitor Performance**:
```bash
# In separate terminal
ros2 topic hz /drone_1/cmd_vel  # Check control loop frequency
ros2 topic echo /drone_1/pose   # Watch position updates
```

3. **Record Data**:
```bash
# Record all topics
ros2 bag record -a -o swarm_test

# Replay later
ros2 bag play swarm_test
```

4. **Visualize in PlotJuggler**:
```bash
sudo apt install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler
```

### Learn More

- Read [TECHNICAL.md](TECHNICAL.md) for in-depth algorithm explanation
- Explore the source code:
  - `swarm_controller.py`: Swarm intelligence logic
  - `drone_node.py`: Kinematic simulation
- Join the ROS2 community forums for help

### Contributing

Found a bug or want to add a feature?
1. Check existing issues on GitHub
2. Create a new issue or pull request
3. Follow the contribution guidelines in README.md

---

## Quick Reference

### Essential Commands

```bash
# Build
colcon build --packages-select multi_drone_pkg

# Source
source ~/ros2_ws/install/setup.bash

# Launch
ros2 launch multi_drone_pkg rviz_swarm_launch.py

# List nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor a topic
ros2 topic echo /drone_1/pose

# View parameters
ros2 param list /swarm_controller
```

### File Locations

```
~/ros2_ws/
├── src/multi_drone_pkg/          # Source code
├── build/multi_drone_pkg/         # Build artifacts
├── install/multi_drone_pkg/       # Installed package
└── log/                           # Build logs
```

---

**Happy Swarming!**

For more help, see [README.md](README.md) or open an issue on GitHub.
