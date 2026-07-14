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

# Create driver. Supported output rates: 1000, 500, 250, 100, 500_16.
driver = Driver(host="192.168.0.100", port=82, streaming_port=54843, output_rate=1000)

# Read a parameter. Values are returned as wire-format hex strings.
parameter = driver.get_parameter(index="0001", subindex="00")
if parameter.error_code == "00":
    print(parameter.param_value)

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

`output_rate="500_16"` selects the sensor's 500 Hz UDP packaged mode. Each UDP packet contains 16 sequential measurements, `sample()` returns those measurements one at a time, and `sample_batch()` returns the 16-measurement packet batch.

`streaming_port` is also written to the sensor as UDP destination port parameter `0x1033/0` before streaming starts. Use a unique port for each sensor when running multiple plain-Ethernet sensors.
