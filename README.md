<div align="center">
  <img src="resources/images/schunk_fts.png" alt="SCHUNK Force-Torque Sensor" style="width: 25%;"/>
  <h1 align="center">SCHUNK Force-Torque Sensor</h1>
</div>

<p align="center">
  <a href="https://opensource.org/licenses/gpl-license">
    <img src="https://img.shields.io/badge/License-GPLv3-orange.svg" alt="License">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor/actions/workflows/industrial_ci_humble_action.yml/badge.svg" alt="build badge humble">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor/actions/workflows/industrial_ci_jazzy_action.yml/badge.svg" alt="build badge jazzy">
  </a>
</p>

---

A ROS2 driver for SCHUNK's new force-torque sensor.

## Under construction
We are intensely working on this driver.
A _beta_ release is planned for mid July '25.

## Build and install
In a new terminal, source your global ROS2 environment, e.g.
```bash
source /opt/ros/humble/setup.bash
```
navigate into your local ROS2 workspace, and build the driver with
```bash
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor.git src/schunk_fts
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
```

## Getting started
Source your local ROS2 workspace with
```bash
source install/setup.bash
```
and start the driver with
```bash
ros2 launch schunk_fts_driver driver.launch.py
```
