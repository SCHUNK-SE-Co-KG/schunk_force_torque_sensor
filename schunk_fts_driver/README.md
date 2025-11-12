# SCHUNK FTS Driver


## Lifecycle behavior

The driver implements a [lifecycle node](https://github.com/ros2/demos/tree/humble/lifecycle) with defined state transitions.
This gives us advanced control over configuration and resetting processes.

### Command line
With built-in features

```bash
ros2 lifecycle get /schunk/driver
ros2 lifecycle set /schunk/driver configure  # activate | deactivate | cleanup | shutdown
```

And with ROS2 services

```bash
ros2 service call /schunk/driver/get_state lifecycle_msgs/srv/GetState '{}'
ros2 service call /schunk/driver/change_state lifecycle_msgs/srv/ChangeState '{transition: {label: configure}}'  # activate | deactivate etc.
```

ROS2 topics

```bash
ros2 topic echo /schunk/driver/data
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /schunk/driver/state
```

## Available Services

The driver provides several services for interacting with the sensor:

### Tare Operations
```bash
# Tare the sensor (set current measurements as zero reference)
ros2 service call /schunk/driver/tare std_srvs/srv/Trigger '{}'

# Reset tare (remove zero reference)
ros2 service call /schunk/driver/reset_tare std_srvs/srv/Trigger '{}'
```

### Tool Settings
Select one of four pre-configured tool settings (0-3). Each tool setting includes a Tool Center Point (TCP) transformation and user-defined overrange limits.

```bash
# Select tool setting 0
ros2 service call /schunk/driver/select_tool_setting schunk_fts_interfaces/srv/SelectToolSetting '{tool_index: 0}'

# Select tool setting 1
ros2 service call /schunk/driver/select_tool_setting schunk_fts_interfaces/srv/SelectToolSetting '{tool_index: 1}'
```

### Noise Reduction Filter
Select a noise reduction filter using rolling average. Filter numbers 0-4 correspond to averaging factors 1, 2, 4, 8, and 16 respectively.

```bash
# No filtering (factor 1)
ros2 service call /schunk/driver/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 0}'

# Medium filtering (factor 4)
ros2 service call /schunk/driver/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 2}'

# Heavy filtering (factor 16)
ros2 service call /schunk/driver/select_noise_filter schunk_fts_interfaces/srv/SelectNoiseFilter '{filter_number: 4}'
```

### Low-Level Commands
For advanced users, generic command and parameter services are available:

```bash
# Send a raw command (hex string)
ros2 service call /schunk/driver/send_command schunk_fts_interfaces/srv/SendCommand '{command_id: "12"}'

# Set a parameter (hex strings for index, subindex, and value)
ros2 service call /schunk/driver/set_parameter schunk_fts_interfaces/srv/SetParameter '{param_index: "0040", param_subindex: "00", param_value: "01020304"}'
```
