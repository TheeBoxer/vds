#!/usr/bin/env bash

readonly BUILD_DIR='build/'

mkdir ${BUILD_DIR}
cd    ${BUILD_DIR}

cmake ../linux/
make

