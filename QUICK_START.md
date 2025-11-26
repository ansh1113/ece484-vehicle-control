# Quick Start Guide

## 🚀 Getting Started in 5 Minutes

### 1. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/ansh1113/ece484-vehicle-control.git mp2
```

### 2. Install Dependencies
```bash
# Install ROS2 packages
sudo apt install ros-humble-ros-control \
                 ros-humble-effort-controllers \
                 ros-humble-joint-state-controller \
                 ros-humble-ackermann-msgs

# Install Python packages
cd ~/ros2_ws/src/mp2
pip install -r requirements.txt
```

### 3. Build
```bash
cd ~/ros2_ws
colcon build --packages-select mp2
source install/setup.bash
```

### 4. Run

**Terminal 1:**
```bash
ros2 launch mp2 gem_vehicle.launch.py
```

**Terminal 2:**
```bash
cd ~/ros2_ws/src/mp2/src
python3 main.py
```

## 🎯 What You Should See

- **Gazebo**: GEM e4 vehicle on race track
- **RViz**: Waypoint visualization
- Vehicle autonomously navigating the track

## 📊 Expected Performance

- Lap time: 69-80 seconds
- All waypoints completed
- Smooth velocity transitions (16.5-27 m/s)

## 🆘 Troubleshooting

**Problem**: "Service not available"
- Wait for Gazebo to fully load
- Check that workspace is sourced

**Problem**: Vehicle not moving
- Verify controller is running
- Check ROS2 topic connections: `ros2 topic list`

See full [README.md](README.md) for detailed documentation.
