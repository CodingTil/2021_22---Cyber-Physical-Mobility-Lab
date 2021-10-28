#!/bin/bash
# exit when any command fails
set -e

PROJECT_DIR=/home/cpm/dev/software
INSTALL_DIR=cpm_lib/thirdparty/install
CMAKE_PREFIX_PATH="${PROJECT_DIR}/${INSTALL_DIR}"

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
cmake --build . -- -j$(nproc)
cd ..
