#/usr/bin/bash
pwd
ls -alF

# Install Rust
sudo apt-get install -y curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"

# Build the FTS dummy and make it available for tests
cd schunk_fts_dummy
export CARGO_TARGET_DIR=/tmp/schunk_fts_dummy
cargo build & echo "Built SCHUNK FTS dummy"
