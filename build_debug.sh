#!/usr/bin/env bash

readonly DEBUG_BUILD_DIR='debug/'

mkdir ${DEBUG_BUILD_DIR}
cd    ${DEBUG_BUILD_DIR}

# debug
cmake ../linux/ -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_TOOLCHAIN_FILE=../linux/arm-linux-gnueabihf.cmake
make

