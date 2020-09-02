# Lidar Pipeline
[![Build Status](https://github.com/rustyducks/lidar_pipeline/workflows/Rust%20Ubuntu%20Build%20and%20Tests/badge.svg)](https://github.com/rustyducks/lidar_pipeline/actions?query=workflow%3A%22Rust+Ubuntu+Build+and+Tests%22)
[![Build Status](https://github.com/rustyducks/lidar_pipeline/workflows/Rust%20ARM%20Cross%20Compiling/badge.svg)](https://github.com/rustyducks/lidar_pipeline/actions?query=workflow%3A%22Rust+ARM+Cross+Compiling%22)

## Installation


### On x86:

1. Install dependencies (for cross-compiling: `sudo apt update && sudo apt install -y -qq gcc-arm-linux-gnueabihf`)
2. Clone repo `git clone https://github.com/rustyducks/lidar_pipeline.git && cd lidar_pipeline`
3. Compile `cargo build` (for cross-compiling: `rustup target add armv7-unknown-linux-gnueabihf && cargo build --target=armv7-unknown-linux-gnueabihf`

### On ARM:

1. Install dependencies
   a. Rust: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`


