# SCHUNK Force-Torque Sensor Dummy

Rust-based sensor simulator for testing without hardware.

## Quick Start

### Install Rust
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### Run
```bash
cd schunk_fts_dummy
cargo run
```

Expected output:
```
SCHUNK Force-Torque Sensor Dummy
Listening on:
  TCP: 127.0.0.1:8082 (commands)
  UDP: 127.0.0.1:52964 -> client:54843 by default (data stream)
```

## Differences from Real Sensor

| Feature | Real Sensor | Dummy |
|---------|-------------|-------|
| Port | 82 (requires root) | 8082 |
| IP | 192.168.0.100 | 127.0.0.1 |
| Data | Real measurements | Random ± noise |
| Commands | Affects sensor | Simulated response only |
| UDP output rate | 1000, 500, 250, 100, 500_16 packaged mode | 1000, 500, 250, 100, 500_16 packaged mode |

The dummy implements `output_rate_udp_ethernet` at parameter `0x1020/0`. Enum values `0`, `1`, `2`, `3`, and `10` select 1000, 500, 250, 100, and 500_16 respectively. The 500_16 mode sends UDP packets at 500 Hz with 16 sequential measurements per packet. The ROS driver publishes those packaged UDP packets as one batch topic message per packet.

The dummy also implements `udp_destination_port` at parameter `0x1033/0`. The default is `54843`; writes to this parameter change the UDP target port used by the dummy stream.

### Run Multiple Dummy Sensors

Each dummy instance needs its own TCP command port. The UDP destination port is
configured by the client driver through parameter `0x1033/0`, so two clients can
stream from two dummy sensors at the same time as long as their driver
`streaming_port` values differ.

```bash
SCHUNK_FTS_DUMMY_TCP_PORT=8082 cargo run
SCHUNK_FTS_DUMMY_TCP_PORT=8083 cargo run
```

Equivalent full bind override:

```bash
SCHUNK_FTS_DUMMY_TCP_ADDR=127.0.0.1:8083 cargo run
```
