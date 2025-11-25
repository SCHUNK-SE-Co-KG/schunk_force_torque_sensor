# SCHUNK FTS Driver

ROS2 lifecycle node for SCHUNK force-torque sensors. Wraps `schunk_fts_library` with ROS2 topics, services, and lifecycle management.

## Quick Start

```bash
# Launch driver
ros2 launch schunk_fts_driver driver.launch.py  # Default: 192.168.0.100
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.1.50  # Custom IP (set via SCHUNK Control Center)

# Activate
ros2 lifecycle set /schunk/driver configure
ros2 lifecycle set /schunk/driver activate

# View data
ros2 topic echo /schunk/driver/data

# Observe state
ros2 topic echo /schunk/driver/state
```

## Lifecycle States

```
Unconfigured -> configure -> Inactive -> activate -> Active
```

- **Unconfigured**: No sensor connection
- **Inactive**: Connected via TCP and UDP, Sensor is streaming
- **Active**: Publishing force-torque data at 1000 Hz

Control via:
```bash
ros2 lifecycle get /schunk/driver
ros2 lifecycle set /schunk/driver configure|activate|deactivate|cleanup
```

## Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/schunk/driver/data` | `geometry_msgs/WrenchStamped` | 1000 Hz | Force-torque data |
| `/schunk/driver/state` | `diagnostic_msgs/DiagnosticStatus` | latched - published on change | Sensor status |

## Services

```bash
# Tare
ros2 service call /schunk/driver/tare std_srvs/srv/Trigger '{}'
ros2 service call /schunk/driver/reset_tare std_srvs/srv/Trigger '{}'

# Tool settings (0-3)
ros2 service call /schunk/driver/select_tool_setting schunk_fts_interfaces/srv/SelectToolSetting '{tool_index: 0}'

# Noise filter (0=none, 1=2x, 2=4x, 3=8x, 4=16x)
ros2 service call /schunk/driver/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 2}'

# Advanced (see Interface Control Document)
ros2 service call /schunk/driver/send_command schunk_fts_interfaces/srv/SendCommand '{command_id: "12"}'
ros2 service call /schunk/driver/set_parameter schunk_fts_interfaces/srv/SetParameter '{param_index: "0040", param_subindex: "00", param_value: "01"}'
```

## Automatic Reconnection

Auto-reconnects on power loss (100ms detection, 1s retry).

## Configuration

```bash
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.0.100 port:=82 streaming_port:=54843
```
