# AhToVik 30-Robot Animated Gazebo Motion Demo

This suite turns the previous static AhToVik visualization into a video-ready Gazebo demonstration.

## What it shows
- 10 drones: D1-D10
- 10 ground robots: G1-G10
- 10 creepers: C1-C10
- Three highlighted task sites:
  - magenta: Scan and Rescue
  - orange: Supply Necessities
  - cyan: Lifting Remains
- Active robots move toward task zones.
- Failed robots freeze.
- Standby robots are promoted using AhToVik-style Q values and move toward the task zone.
- A CSV log is saved at `~/ahtovik_30_robot_motion_log.csv`.

## Install
Copy/extract the `ahtovik_gazebo` folder into your ROS2 workspace `src` folder.

```bash
cd ~/ros2_ws/src
unzip ahtovik_30_robot_motion_demo_suite.zip
```

If asked to overwrite existing `ahtovik_gazebo`, choose `A` or remove the old folder first:

```bash
rm -rf ~/ros2_ws/src/ahtovik_gazebo
unzip ahtovik_30_robot_motion_demo_suite.zip
```

## Build and source

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Check package:

```bash
ros2 pkg list | grep ahtovik
```

## Launch full animated demo

```bash
ros2 launch ahtovik_gazebo ahtovik_30_robot_motion_demo.launch.py
```

Gazebo will open and the AhToVik motion node will run automatically.

## If Gazebo opens but robots do not move
Open another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ahtovik_gazebo ahtovik_motion_demo
```

## Record video
Install OBS:

```bash
sudo apt update
sudo apt install obs-studio
```

Then record the Gazebo window.

## Important note
This uses Gazebo Classic service `/gazebo/set_entity_state`. The world loads `libgazebo_ros_init.so`, so the service should be available after the launch.
