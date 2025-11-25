# SCHUNK FTS Library

Low-level Python library for SCHUNK force-torque sensor communication. Handles TCP commands and UDP data streaming.

## Installation

### With ROS2
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select schunk_fts_library
```

### Standalone
```bash
pip install .
```

## Quick Start

```python
from schunk_fts_library.driver import Driver
import time

# Create driver
driver = Driver(host="192.168.0.100", port=82, streaming_port=54843)

# Start streaming with auto-reconnect
driver.streaming_on()
time.sleep(0.1)

# Read data
for _ in range(10000):
    data = driver.sample()
    if data:
        print(f"Force: [{data['fx']:.2f}, {data['fy']:.2f}, {data['fz']:.2f}] N")
        print(f"Torque: [{data['tx']:.2f}, {data['ty']:.2f}, {data['tz']:.2f}] Nm")

# Stop streaming
driver.streaming_off()
```
