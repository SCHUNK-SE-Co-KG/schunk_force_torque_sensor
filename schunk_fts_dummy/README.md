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
  UDP: 127.0.0.1:52964 -> client:54843 (data stream)
```

## Differences from Real Sensor

| Feature | Real Sensor | Dummy |
|---------|-------------|-------|
| Port | 82 (requires root) | 8082 |
| IP | 192.168.0.100 | 127.0.0.1 |
| Data | Real measurements | Random ± noise |
| Commands | Affects sensor | Simulated response only |
