# SCHUNK FTS Driver

ROS2 lifecycle node for SCHUNK force-torque sensors. Wraps `schunk_fts_library` with ROS2 topics, services, and lifecycle management.

## Quick Start

```bash
# Launch driver
ros2 launch schunk_fts_driver driver.launch.py  # Default: 192.168.0.100
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.1.50  # Custom IP (set via SCHUNK Control Center)
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.1.50 streaming_port:=54844  # Change the UDP receive port
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.1.50 output_rate:=500_16

# Activate
ros2 lifecycle set /schunk/fts configure
ros2 lifecycle set /schunk/fts activate

# View data
ros2 topic echo /schunk/fts/data

# Observe state
ros2 topic echo /schunk/fts/state
```

## Lifecycle States

```
Unconfigured -> configure -> Inactive -> activate -> Active
```

- **Unconfigured**: No sensor connection
- **Inactive**: Connected via TCP and UDP, Sensor is streaming
- **Active**: Publishing force-torque data at the configured sensor output rate

Control via:
```bash
ros2 lifecycle get /schunk/fts
ros2 lifecycle set /schunk/fts configure|activate|deactivate|cleanup
```

## Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/schunk/fts/data` | `geometry_msgs/WrenchStamped` for `1000`, `500`, `250`, `100`; `schunk_fts_interfaces/WrenchStampedBatch` for `500_16` | Sensor output rate, or 500 Hz in `500_16` | Force-torque data. The topic type depends on `output_rate`; `500_16` publishes one message containing 16 timestamp-spaced samples per UDP packet. |
| `/schunk/fts/state` | `diagnostic_msgs/DiagnosticStatus` | latched - published on change | Sensor status |

## Services

```bash
# Tare
ros2 service call /schunk/fts/tare std_srvs/srv/Trigger '{}'
ros2 service call /schunk/fts/reset_tare std_srvs/srv/Trigger '{}'

# Tool settings (0-3)
ros2 service call /schunk/fts/select_tool_setting schunk_fts_interfaces/srv/SelectToolSetting '{tool_index: 0}'

# Noise filter (0=none, 1=2x, 2=4x, 3=8x, 4=16x)
ros2 service call /schunk/fts/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 2}'

# Advanced (see Interface Control Document)
ros2 service call /schunk/fts/send_command schunk_fts_interfaces/srv/SendCommand '{command_id: "12"}'
ros2 service call /schunk/fts/set_parameter schunk_fts_interfaces/srv/SetParameter '{param_index: "0040", param_subindex: "00", param_value: "01"}'
```

## Automatic Reconnection

Auto-reconnects on power loss (100ms detection, 1s retry).

## Configuration

```bash
ros2 launch schunk_fts_driver driver.launch.py host:=192.168.0.100 port:=82 streaming_port:=54843 output_rate:=1000
```

Supported `output_rate` values are `1000`, `500`, `250`, `100`, and `500_16`. The default is `1000`.

Changing `output_rate` can change the `/schunk/fts/data` message type. The normal modes (`1000`, `500`, `250`, `100`) publish `geometry_msgs/WrenchStamped`. In `500_16` mode, the sensor sends 500 UDP packets per second and each packet carries 16 sequential measurements, so the driver publishes `schunk_fts_interfaces/WrenchStampedBatch` at 500 Hz with timestamps spread across the packet period.

`streaming_port` is the driver's local UDP receive port. Change it by passing a different value on the launch command, for example `streaming_port:=54844`. It is written to the sensor as UDP destination port parameter `0x1033/0` during lifecycle configure. Set a distinct `streaming_port` for each sensor instance.
