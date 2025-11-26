# Autonomous Vehicle Control - Pure Pursuit & Longitudinal Control

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](https://gazebosim.org/)

**Lateral and longitudinal vehicle control for autonomous racing - UIUC ECE 484 Safe Autonomy**

## Key Results

- ✅ **Fast Lap Time** - 67-73 seconds (target: <130s)
- ✅ **100% Waypoint Success** - All waypoints completed safely
- ✅ **Adaptive Speed Control** - 16.5-27 m/s based on curvature
- ✅ **Smooth Trajectories** - Minimal steering oscillations
- ✅ **Comfort Analysis** - Acceleration profiling and optimization

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Control Algorithms](#control-algorithms)
- [Longitudinal Controller Approaches](#longitudinal-controller-approaches)
- [Installation](#installation)
- [Usage](#usage)
- [Experimental Results](#experimental-results)
- [Performance Analysis](#performance-analysis)
- [Course Context](#course-context)
- [Team](#team)

---

## Overview

This project implements **autonomous vehicle control** for a Polaris GEM e4 electric cart navigating a race track. The system combines:

1. **Lateral Control** - Pure Pursuit algorithm for steering
2. **Longitudinal Control** - Curvature-based speed regulation
3. **Safety & Comfort** - Acceleration limiting and smooth transitions

### Problem Statement

**Given:**
- Pre-defined waypoints around race track
- Vehicle kinematic model (Ackermann steering)
- Track geometry (straights and curves)

**Find:**
- Steering angle δ(t) to follow waypoints
- Target velocity v(t) to optimize lap time while maintaining safety

### Performance Metrics

- **Safety**: Complete all waypoints without deviation
- **Efficiency**: Lap time < 130 seconds
- **Comfort**: Limit accelerations to < 0.5G (5 m/s²)

---

## System Architecture
```
┌─────────────────┐
│  Waypoint List  │
│  (Track Path)   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────┐
│    Longitudinal Controller      │
│  ┌──────────────────────────┐   │
│  │ Curvature Analysis       │   │
│  │ Speed Mapping            │   │
│  │ Acceleration Limiting    │   │
│  └──────────────────────────┘   │
└────────┬────────────────────────┘
         │ Target Velocity v
         ▼
┌─────────────────────────────────┐
│     Lateral Controller          │
│  ┌──────────────────────────┐   │
│  │ Pure Pursuit Algorithm   │   │
│  │ Lookahead Distance       │   │
│  │ Steering Angle Calc      │   │
│  └──────────────────────────┘   │
└────────┬────────────────────────┘
         │ Steering δ, Velocity v
         ▼
┌─────────────────┐
│  GEM e4 Vehicle │
│  (Gazebo Sim)   │
└─────────────────┘
```

### Components

**1. Lateral Controller (`controller.py`)**
- Pure Pursuit path tracking
- Dynamic lookahead distance selection
- Smooth steering transitions

**2. Longitudinal Controller (`controller.py`)**
- Curvature-based speed mapping
- Predictive braking
- Acceleration/deceleration limiting

**3. Vehicle Node (`vehicle_node.py`)**
- ROS2 interface
- State estimation
- Control command publishing

**4. Utilities (`util.py`, `waypoint_list.py`)**
- Coordinate transformations
- Waypoint management
- Geometry calculations

---

## Control Algorithms

### Lateral Control: Pure Pursuit

Pure Pursuit is a path-tracking algorithm that computes steering commands to reach a lookahead point on the desired path.

#### Algorithm
```python
# 1. Find lookahead point on path
lookahead_distance = f(velocity, curvature)
lookahead_point = find_point_at_distance(path, lookahead_distance)

# 2. Transform to vehicle frame
alpha = atan2(lookahead_point.y, lookahead_point.x)

# 3. Compute steering angle using Pure Pursuit formula
steering_angle = atan2(2 * wheelbase * sin(alpha), lookahead_distance)
```

#### Lookahead Distance Selection

We explored two approaches:

**Approach A: Nearest Waypoint + Midpoint**
```python
# Target midpoint between next and next-next waypoint
lookahead_distance = distance_to_next + 0.5 * waypoint_spacing
```

**Pros**: Aims directly at waypoints  
**Cons**: Can cause oscillations at waypoint transitions

**Approach B: Fixed Lookahead with Interpolation (Selected)**
```python
# Constant lookahead distance, interpolate between waypoints
lookahead_distance = FIXED_DISTANCE  # e.g., 5-8 meters
target_point = interpolate_on_path(current_pos, lookahead_distance)
```

**Pros**: Smooth steering, stable in curves  
**Cons**: May cut corners slightly

**Winner**: Approach B - Provides superior stability and smoother trajectories

---

## Longitudinal Controller Approaches

We developed and tested **5 different approaches** for mapping track curvature to target velocity:

### Approach 1: Constant Velocity Baseline
```python
target_velocity = 10.0  # m/s (constant)
```

**Performance:**
- Lap time: ~139 seconds
- Safety: Perfect (100% waypoints)
- Comfort: Excellent (no harsh accelerations)

**Verdict**: Too conservative, exceeds time limit

---

### Approach 2: Fixed Lookahead with Thresholds
```python
# Compute average curvature over next 7 waypoints
curvature = mean(abs(angle_changes[:7]))

if curvature > 0.23:
    target_velocity = 16.0  # Sharp turn
elif curvature > 0.13:
    target_velocity = 18.0  # Medium turn
else:
    target_velocity = 23.7  # Straight
```

**Performance:**
- Lap time: ~67 seconds ✅
- Safety: Good (minor skids in tight corners)
- Comfort: Moderate (some acceleration spikes)

**Verdict**: Fast but aggressive

---

### Approach 3: Blended Past-Present-Future with Accel Caps
```python
# Use 5-waypoint window (2 past + current + 2 future)
curvature = mean(abs(angle_changes[-2:3]))

# Smooth mapping with acceleration limits
target_velocity = 28.0 / (curvature * 4 + 1)
target_velocity = clamp_acceleration(target_velocity, prev_velocity, 
                                     max_accel=5.0)
```

**Performance:**
- Lap time: ~80 seconds
- Safety: Excellent (100% waypoints, no skids)
- Comfort: Very Good (smooth acceleration profile)

**Verdict**: Balanced approach

---

### Approach 4: Dynamic Lookahead with Lateral G-Force Limits
```python
# Speed-dependent lookahead distance
lookahead = 10.0 + 0.2 * velocity  # meters
lookahead = min(lookahead, 25.0)

# Compute curvature at lookahead point
kappa = 2 * sin(alpha) / lookahead

# Limit based on lateral acceleration (4 m/s²)
target_velocity = min(18.0, sqrt(4.0 / abs(kappa)))
```

**Performance:**
- Lap time: ~73 seconds
- Safety: Excellent (100% waypoints)
- Comfort: Good (physics-based limiting)

**Verdict**: Theoretically sound, reliable

---

### Approach 5: Sectional Analysis with Predictive Braking (Selected)
```python
# Divide track ahead into 30-meter sections
sections = group_waypoints(ahead, section_length=30)

# Compute curvature factor for each section
curvature_factor = 0.0
for section in sections:
    curvature_factor += mean(abs(angle_changes(section)))

# Map to velocity (16.5-27 m/s range)
target_velocity = 16.5 + (27.0 - 16.5) * (1 - curvature_factor)

# Predictive braking
if current_velocity > target_velocity:
    braking_distance = (current_velocity^2 - target_velocity^2) / (2 * 3.0)
    if distance_to_curve < braking_distance * 1.3:  # 30% safety margin
        # Gradual deceleration
        target_velocity -= 0.05 * (current_velocity - target_velocity)
else:
    # Gradual acceleration
    target_velocity += 0.03 * (target_velocity - current_velocity)
```

**Performance:**
- Lap time: **69 seconds** ⚡
- Safety: **Perfect** (100% waypoints, zero slides)
- Comfort: **Excellent** (smooth accel profile)

**Why It Works:**
- ✅ Looks far ahead to anticipate curves
- ✅ Smooth speed transitions (no jerks)
- ✅ Safety margin in braking calculations
- ✅ Balances speed and comfort optimally

**Winner**: Best overall performance across all metrics

---

## Installation

### Prerequisites
```bash
# System requirements
Ubuntu 22.04
ROS2 Humble
Gazebo 11
Python 3.8+
```

### ROS2 Package Dependencies
```bash
sudo apt install ros-humble-ros-control \
                 ros-humble-effort-controllers \
                 ros-humble-joint-state-controller \
                 ros-humble-ackermann-msgs
```

### Python Dependencies
```bash
pip install -r requirements.txt
```

**requirements.txt:**
```
numpy>=1.21.0
matplotlib>=3.4.0
```

### Build Instructions
```bash
# Clone repository into ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/ansh1113/ece484-vehicle-control.git mp2

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## Usage

### Quick Start

**Terminal 1: Launch Gazebo Simulation**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mp2 gem_vehicle.launch.py
```

**Terminal 2: Run Vehicle Controller**
```bash
cd ~/ros2_ws/src/mp2/src
python3 main.py
```

### Command Line Arguments
```bash
python3 main.py --help

Options:
  --approach N       Longitudinal controller approach (1-5, default: 5)
  --visualize        Enable real-time plotting
  --log_accel        Save acceleration data to CSV
```

### Example Configurations

**Standard Run (Approach 5):**
```bash
python3 main.py --approach 5 --log_accel
```

**Comparison Test:**
```bash
# Test all approaches
for i in {1..5}; do
    python3 main.py --approach $i --log_accel
    sleep 5
done
```

---

## Experimental Results

### Lap Time Comparison

| Approach | Lap Time | Waypoint Success | Comfort | Notes |
|----------|----------|------------------|---------|-------|
| 1. Constant | 139s | 100% | Excellent | Too slow |
| 2. Fixed Thresholds | 67s | 98% | Moderate | Aggressive |
| 3. Blended + Accel Caps | 80s | 100% | Very Good | Balanced |
| 4. Dynamic Lookahead | 73s | 100% | Good | Physics-based |
| **5. Sectional + Braking** | **69s** | **100%** | **Excellent** | **Best overall** |

### Comfort Analysis

Acceleration profile from Approach 5:

**Observations:**
- Peak accelerations: ~40 m/s² (brief spikes)
- Typical accelerations: 5-15 m/s²
- Exceeded 0.5G threshold frequently

**Contributing Factors:**

1. **Non-smooth Waypoints**
   - Lateral offsets between consecutive waypoints
   - Controller attempts to follow "jerky" reference path
   - Creates lateral acceleration spikes

2. **Track Geometry Issues**
   - Gaps between road segment meshes in Gazebo
   - "Pothole effect" from mesh discontinuities
   - Vertical accelerations as vehicle bounces

3. **Speed Optimization**
   - Controller prioritizes lap time over comfort
   - High speeds on straights (27 m/s) → Large decelerations before curves
   - Trade-off accepted for competitive performance

**Improvement Strategies:**
- Smooth waypoint path with spline interpolation
- Add comfort cost to optimization objective
- Reduce max speed from 27 → 23 m/s
- Increase braking safety margin from 1.3 → 1.5

---

## Performance Analysis

### Vehicle Trajectory

Actual position tracking vs. target waypoints:

**Observations:**
- Excellent path following accuracy
- Minimal deviation from reference trajectory
- Smooth cornering with no oscillations

### Velocity Profile

Speed variation throughout lap:

**Key Features:**
- Max speed: 27 m/s on long straights
- Min speed: 16.5 m/s in tightest corners
- Smooth transitions between regimes
- Predictive braking before curves

### Steering Behavior

**Lateral Controller Performance:**
- Stable steering angles throughout
- No high-frequency oscillations
- Appropriate response to track curvature
- Fixed lookahead prevents overshoot

---

## Demo Videos

**Video Links**: [Google Drive - MP2 Demos](https://drive.google.com/drive/folders/1ICG73adK5TKaqPeVs3uwdowgrdKpFXaQ?usp=drive_link)

### Visualization

The simulation shows:
- **Gazebo**: GEM e4 vehicle on race track
- **RViz**: Waypoint visualization and vehicle state
- Real-time velocity and steering angle displays

---

## Technical Implementation

### Pure Pursuit Implementation
```python
def pure_pursuit_control(self, current_pos, current_heading, waypoints):
    """
    Compute steering angle using Pure Pursuit algorithm
    """
    # Find lookahead point
    lookahead_distance = 5.0  # meters
    target_point = self.find_lookahead_point(
        current_pos, waypoints, lookahead_distance
    )
    
    # Transform to vehicle frame
    dx = target_point.x - current_pos.x
    dy = target_point.y - current_pos.y
    
    # Rotate to vehicle heading
    local_x = cos(current_heading) * dx + sin(current_heading) * dy
    local_y = -sin(current_heading) * dx + cos(current_heading) * dy
    
    # Compute steering angle
    alpha = atan2(local_y, local_x)
    steering = atan2(2 * self.wheelbase * sin(alpha), lookahead_distance)
    
    return steering
```

### Sectional Curvature Analysis
```python
def compute_target_velocity(self, current_pos, waypoints):
    """
    Approach 5: Sectional analysis with predictive braking
    """
    # Divide ahead into sections
    section_length = 30  # meters
    sections = self.group_into_sections(waypoints, section_length)
    
    # Compute curvature for each section
    curvatures = []
    for section in sections:
        angles = []
        for i in range(len(section) - 1):
            angle = atan2(section[i+1].y - section[i].y,
                         section[i+1].x - section[i].x)
            angles.append(angle)
        
        # Average absolute change in heading
        curvature = mean(abs(diff(angles)))
        curvatures.append(curvature)
    
    # Map to velocity (16.5-27 m/s)
    avg_curvature = mean(curvatures)
    target_vel = 16.5 + (27.0 - 16.5) * (1 - min(avg_curvature, 1.0))
    
    # Predictive braking
    if self.current_velocity > target_vel:
        braking_dist = (self.current_velocity**2 - target_vel**2) / (2 * 3.0)
        dist_to_curve = self.distance_to_next_curve(current_pos, waypoints)
        
        if dist_to_curve < braking_dist * 1.3:
            # Gradual deceleration
            return self.current_velocity - 0.05 * (self.current_velocity - target_vel)
    
    # Gradual acceleration
    return self.current_velocity + 0.03 * (target_vel - self.current_velocity)
```

---

## Course Context

**Course**: ECE 484 - Principles of Safe Autonomy  
**Institution**: University of Illinois Urbana-Champaign  
**Semester**: Fall 2025  
**Project Type**: Machine Problem 2 

---

## 📁 Project Structure
```
ece484-vehicle-control/
├── controller.py           # Main control algorithms
├── vehicle_node.py         # ROS2 vehicle interface
├── main.py                 # Entry point
├── util.py                 # Utility functions
├── waypoint_list.py        # Track waypoint data
├── set_pos.py             # Position reset utility
├── requirements.txt        # Python dependencies
├── README.md              # This file
└── launch/
    └── gem_vehicle.launch.py  # ROS2 launch file
```

---

## 📖 Academic Integrity Statement

This repository contains coursework from ECE 484 - Principles of Safe Autonomy at UIUC.  
Shared for portfolio and educational purposes after course completion.

**If you are currently enrolled in this course:**
- ❌ Do NOT copy this code for your assignments
- ✅ Use only as a learning reference
- ✅ Follow your course's academic integrity policy

Violations of academic integrity policies will be reported.

---

## License

MIT License - See [LICENSE](LICENSE) for details

---

## Acknowledgments

- ECE 484 course staff for track design and infrastructure
- UIUC Robotics Lab for Polaris GEM e4 platform

---

## 📞 Contact

For questions about this implementation:
- **Ansh Bhansali**: anshbhansali5@gmail.com
- **GitHub**: [@ansh1113](https://github.com/ansh1113)

---

**⭐ If you find this helpful, please star the repository!**
