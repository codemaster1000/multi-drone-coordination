# Multi-Drone Swarm System

A ROS2 Humble package for simulating intelligent drone swarms with dynamic collision avoidance and coordinated flight patterns.

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python 3.10](https://img.shields.io/badge/Python-3.10-green)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

## Features

- **Boids-based Flocking**: Separation, cohesion, and alignment behaviors
- **Circular Formation**: Leader follows circular path, followers maintain formation
- **Collision Avoidance**: Dynamic obstacle avoidance with hysteresis
- **RViz Visualization**: Real-time 3D visualization of swarm behavior
- **Lightweight**: Kinematic simulation without physics engine overhead
- **State Machine**: IDLE, FLOCK, FOLLOW, AVOID, and SAFE states

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Swarm Controller                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   State      │→ │   Motion     │→ │  Actuation   │   │
│  │ Estimation   │  │  Generation  │  │   & Limits   │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────┘
          ↓ cmd_vel                    ↑ pose
┌─────────────────────────────────────────────────────────┐
│              5 x Drone Nodes (Kinematic)                │
│   Drone 1 (Leader) │ Drone 2 │ Drone 3 │ Drone 4 │ 5    │
└─────────────────────────────────────────────────────────┘
          ↓ pose
┌─────────────────────────────────────────────────────────┐
│                    RViz2                                │
│              Real-time 3D Visualization                 │
└─────────────────────────────────────────────────────────┘
```

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- RViz2

### Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/codemaster1000/multi-drone-coordination multi_drone_pkg

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select multi_drone_pkg

# Source
source install/setup.bash
```

### Run the Swarm

```bash
# Launch with RViz visualization (recommended)
ros2 launch multi_drone_pkg rviz_swarm_launch.py

# Launch without RViz (lightweight)
ros2 launch multi_drone_pkg swarm_launch.py
```

## Usage Examples

### Default Configuration (5 Drones, Pentagon Formation)

```bash
ros2 launch multi_drone_pkg rviz_swarm_launch.py
```

### Custom Parameters

```bash
# Faster circular motion with larger radius
ros2 launch multi_drone_pkg rviz_swarm_launch.py \
  leader_speed:=2.0 \
  leader_radius:=10.0

# More aggressive separation
ros2 launch multi_drone_pkg rviz_swarm_launch.py \
  separation_weight:=2.5 \
  collision_threshold:=0.5
```

### Manual Control

Run individual components separately for debugging:

```bash
# Terminal 1: Start drones
ros2 run multi_drone_pkg drone_node --ros-args -r __ns:=/drone_1 -p drone_id:=1 -p x_start:=0.0 -p y_start:=0.0 -p z_start:=1.0

# Terminal 2: Start swarm controller
ros2 run multi_drone_pkg swarm_controller --ros-args -p num_drones:=5 -p leader_id:=1

# Terminal 3: Visualize
rviz2 -d src/multi_drone_pkg/rviz/swarm.rviz
```

## Configuration

All swarm parameters can be tuned in `launch/rviz_swarm_launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `collision_threshold` | 0.3m | Minimum safe distance between drones |
| `separation_weight` | 1.5 | Strength of repulsion force |
| `cohesion_weight` | 1.0 | Attraction to swarm center |
| `alignment_weight` | 1.0 | Tendency to match neighbors' velocity |
| `neighbor_distance` | 15.0m | Detection radius for neighbors |
| `leader_speed` | 1.0 m/s | Circular path velocity |
| `leader_radius` | 5.0m | Circle radius for FOLLOW mode |
| `max_speed` | 2.0 m/s | Maximum drone velocity |
| `max_acceleration` | 1.0 m/s² | Acceleration limit |

## State Machine

The swarm operates with 5 distinct states:

1. **IDLE**: Waiting for neighbors to appear
2. **FLOCK**: Boids-based flocking behavior
3. **FOLLOW**: Leader flies circle, followers maintain formation
4. **AVOID**: Emergency collision avoidance (highest priority)
5. **SAFE**: Fail-safe mode when too few drones are active

State transitions happen automatically based on:
- Neighbor proximity
- Leader presence
- Collision risk
- System health

## Project Structure

```
multi_drone_pkg/
├── launch/
│   ├── rviz_swarm_launch.py      # Main launch (with RViz)
│   └── swarm_launch.py            # Lightweight launch (no RViz)
├── multi_drone_pkg/
│   ├── drone_node.py              # Kinematic drone simulation
│   └── swarm_controller.py        # Centralized swarm intelligence
├── rviz/
│   └── swarm.rviz                 # RViz configuration
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── README.md
├── GETTING_STARTED.md
├── LICENSE
├── TECHNICAL.md
├── package.xml
└── setup.py
```

## Documentation

- **[Getting Started Guide](GETTING_STARTED.md)**: Step-by-step tutorial for beginners
- **[Technical Documentation](TECHNICAL.md)**: In-depth explanation of algorithms and math

## Troubleshooting

### Drones oscillate between FOLLOW and AVOID

Increase the collision threshold or reduce drone density:
```python
"collision_threshold": 0.5,  # Increase from 0.3
"neighbor_distance": 20.0,   # Increase from 15.0
```

### Swarm drifts away

Remove or reduce bias velocity:
```python
"bias_velocity_x": 0.0,
"bias_velocity_y": 0.0,
"bias_velocity_z": 0.0,
```

### Drones don't maintain formation

Increase cohesion and alignment:
```python
"cohesion_weight": 2.0,    # Increase from 1.0
"alignment_weight": 2.0,   # Increase from 1.0
```

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Testing

```bash
# Run Python style checks
cd ~/ros2_ws
colcon test --packages-select multi_drone_pkg
colcon test-result --verbose
```

## Performance

- **CPU Usage**: ~5-10% per drone (Intel i5-8250U)
- **Memory**: ~50MB total for 5-drone swarm
- **Update Rate**: 10 Hz control loop, 1 Hz status reporting

## Roadmap

- [ ] Add path planning capabilities
- [ ] Implement obstacle avoidance (static obstacles)
- [ ] Support for heterogeneous swarms
- [ ] Machine learning-based formation optimization
- [ ] Hardware deployment with real drones
- [ ] Multi-swarm coordination

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors

- Nafees - Initial work

## Acknowledgments

- Craig Reynolds for the original Boids algorithm (1986)
- ROS2 community for excellent documentation
- Open Robotics for RViz2

## References

1. Reynolds, C. W. (1987). Flocks, herds and schools: A distributed behavioral model. *ACM SIGGRAPH Computer Graphics*, 21(4), 25-34.
2. Olfati-Saber, R. (2006). Flocking for multi-agent dynamic systems: Algorithms and theory. *IEEE Transactions on automatic control*, 51(3), 401-420.
3. Vásárhelyi, G., et al. (2018). Optimized flocking of autonomous drones in confined environments. *Science Robotics*, 3(20).

## Citation

If you use this software in your research, please cite:

```bibtex
@software{multi_drone_swarm,
  author = {Nafees},
  title = {Multi-Drone Swarm System for ROS2},
  year = {2026},
  url = {https://github.com/codemaster1000/multi-drone-coordination}
}
```

## Support

For questions and support:
- Open an issue on GitHub
- Email: nafeesrealnazar2937@gmail.com

---

**If you find this project useful, please consider giving it a star!**
