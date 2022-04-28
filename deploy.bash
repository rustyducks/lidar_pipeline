#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail
set -o xtrace

export PKG_CONFIG_DIR=
export PKG_CONFIG_LIBDIR=/usr/lib/arm-linux-gnueabihf/pkgconfig
export PKG_CONFIG_SYSROOT_DIR=/usr/lib/arm-linux-gnueabihf
export PKG_CONFIG_ALLOW_CROSS=1

readonly TARGET_HOST=pi@dalek
readonly TARGET_PATH=/home/pi/
readonly TARGET_ARCH=armv7-unknown-linux-gnueabihf
readonly SOURCE_PATH=./target/${TARGET_ARCH}/release/lidar_pipeline

cargo build --release --target=${TARGET_ARCH}
rsync ${SOURCE_PATH} ${TARGET_HOST}:${TARGET_PATH}
rsync obstacles_lidar_mask.yaml ${TARGET_HOST}:${TARGET_PATH}
ssh -t ${TARGET_HOST} ${TARGET_PATH}

