#/usr/bin/bash
pwd
ls -alF

# Install Rust
sudo apt-get install -y curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"

# Build and start the dummy
cd schunk_fts_dummy
export CARGO_TARGET_DIR=/tmp/cargo-target
cargo run && echo "Started SCHUNK FTS dummy"
