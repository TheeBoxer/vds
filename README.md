# Variable Drag System
[![Build Status](https://travis-ci.org/nolanholden/vds.svg?branch=master)](https://travis-ci.org/nolanholden/vds)
[![Build status](https://ci.appveyor.com/api/projects/status/github/nolanholden/vds?branch=master&svg=true)](https://ci.appveyor.com/project/nolanholden/vds)
[![Coverage Status](https://coveralls.io/repos/github/nolanholden/vds/badge.svg?branch=master)](https://coveralls.io/github/nolanholden/vds?branch=master)

## Build Configuration
To build, use the CMake flag `-DCMAKE_TOOLCHAIN_FILE=arm-linux-gnueabihf.cmake`. For example, clone and build a debug (cross-compiled) executable like so:
```sh
git clone https://github.com/nolanholden/vds.git
mkdir vds/linux/build && cd vds/linux/build
cmake ../ -DCMAKE_TOOLCHAIN_FILE=../arm-linux-gnueabihf.cmake
# or, to debug:
# cmake ../ -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_TOOLCHAIN_FILE=../arm-linux-gnueabihf.cmake
make
# there should now be an executable `vds` in the immediate directory
```

## Prerequisites
+ CMake
+ GNU ARM Linux Embedded ABI with Hard Float CXX compiler (`arm-linux-eabihf-g++`)
```sh
sudo apt-get update
sudo apt-get install g++-arm-linux-gnueabihf
```

# Linux Computer Configuration
+ To support SPI comm, execute the `linux/config/spi/setup_spi/` bash script with root privileges on the target machine.
```sh
sudo setup_spi
```
