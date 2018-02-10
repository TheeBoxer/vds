#!/usr/bin/env bash

readonly BUILD_DIR='build/'

mkdir ${BUILD_DIR}
cd    ${BUILD_DIR}

# debug
cmake ../linux/ -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_TOOLCHAIN_FILE=../linux/arm-linux-gnueabihf.cmake
make
