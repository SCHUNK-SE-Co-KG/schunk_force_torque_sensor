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

A ROS2 driver for SCHUNK force-torque sensors with 1000 Hz data streaming, automatic reconnection, and lifecycle management.

## Features

- 1000 Hz force-torque data streaming via UDP
- Automatic reconnection e.g. on power loss
- ROS2 lifecycle node with controlled state transitions
- Tare operations and tool settings (0-3)
- Adjustable noise filters (0-4)
- Rust-based sensor simulator for testing without hardware

## Quick Start

### For Users

#### 1. Install
```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor.git src/schunk_fts
rosdep install --from-paths src --ignore-src -y
colcon build
```

#### 2. Run
```bash
source install/setup.bash
ros2 launch schunk_fts_driver driver.launch.py  # Default: 192.168.0.100
```

#### 3. Activate
```bash
ros2 lifecycle set /schunk/driver configure
ros2 lifecycle set /schunk/driver activate
ros2 topic echo /schunk/driver/data
```

### For Developers

#### Setup
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh  # Install Rust
pip install pytest pre-commit mypy black flake8
pre-commit install
```

#### Run Simulator
```bash
cd schunk_fts_dummy && cargo run  # Terminal 1
ros2 launch schunk_fts_driver driver.launch.py host:=127.0.0.1 port:=8082  # Terminal 2
```

#### Test
Tests require either a connected real sensor or a running simulator. Tests will automatically detect which is connected. Running both at the same time is not recommended. IP-addresses can be adjusted in [fixtures.py](schunk_fts_library/schunk_fts_library/fixtures.py).
```bash
pytest  # Run all tests
```

## Project Structure

```
schunk_force_torque_sensor/
├── schunk_fts_library/       # Low-level Python library
├── schunk_fts_driver/        # ROS2 lifecycle node wrapper
├── schunk_fts_interfaces/    # Custom ROS2 service definitions
└── schunk_fts_dummy/         # Rust-based sensor simulator
```

See individual package READMEs for detailed documentation:
- [schunk_fts_library](schunk_fts_library/README.md) - Library API and usage
- [schunk_fts_driver](schunk_fts_driver/README.md) - Driver configuration and services
- [schunk_fts_interfaces](schunk_fts_interfaces/readme.md) - Service interface definitions
- [schunk_fts_dummy](schunk_fts_dummy/README.md) - Simulator usage

## Common Operations

```bash
# Tare sensor
ros2 service call /schunk/driver/tare std_srvs/srv/Trigger '{}'

# Select tool setting (0-3)
ros2 service call /schunk/driver/select_tool_setting schunk_fts_interfaces/srv/SelectToolSetting '{tool_index: 0}'

# Set noise filter (0=none, 1=2x, 2=4x, 3=8x, 4=16x)
ros2 service call /schunk/driver/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 2}'

# Lifecycle control
ros2 lifecycle set /schunk/driver configure
ros2 lifecycle set /schunk/driver activate
```

## Troubleshooting

**Cannot connect**: Check `ping 192.168.0.100`, verify sensor is powered, check firewall (TCP:82, UDP:54843)

**Low rate**: Check CPU load, network quality, ensure driver is ACTIVE
For high-frequency RT applications, consider using a industrial Ethernet variant of the sensor as they support 8000 Hz.

**Wrong data**: Tare the sensor, check tool setting, verify status topic

**Build fails**: Source ROS2, run `rosdep install`, try clean build

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/schunk/driver/data` | `geometry_msgs/WrenchStamped` | Force-torque measurements at 1000 Hz |
| `/schunk/driver/state` | `diagnostic_msgs/DiagnosticStatus` | Sensor status and diagnostics |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/schunk/driver/tare` | `std_srvs/Trigger` | Zero the sensor data|
| `/schunk/driver/reset_tare` | `std_srvs/Trigger` | Remove tare offset |
| `/schunk/driver/select_tool_setting` | `schunk_fts_interfaces/SelectToolSetting` | Select tool configuration (0-3) |
| `/schunk/driver/select_noise_filter` | `schunk_fts_interfaces/SelectNoiseFilter` | Select noise filter (0-4) |
| `/schunk/driver/send_command` | `schunk_fts_interfaces/SendCommand` | Send raw command (advanced) |
| `/schunk/driver/set_parameter` | `schunk_fts_interfaces/SetParameter` | Set sensor parameter (advanced) |

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Support

[GitHub Issues](https://github.com/SCHUNK-SE-Co-KG/schunk_force_torque_sensor/issues) | SCHUNK SE & Co. KG
