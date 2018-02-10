#!/usr/bin/env bash

readonly BUILD_DIR='build/'

mkdir ${BUILD_DIR}
cd    ${BUILD_DIR}

cmake ../linux/ -DCMAKE_CXX_COMPILER=/usr/bin/arm-linux-gnueabihf-g++ -DCMAKE_CC_COMPILER=/usr/bin/arm-linux-gnueabihf-gcc
make
