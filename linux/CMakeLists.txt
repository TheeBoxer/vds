cmake_minimum_required(VERSION 2.8)
project(vds)

# beaglebone-c
project(bbc)
file(GLOB_RECURSE bbc_sources lib/beaglebone-c/src/*.cc)
add_library(bbc ${bbc_sources})
target_include_directories(bbc INTERFACE lib/beaglebone-c/include/)
set_target_properties(bbc PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/beaglebone-c/include/)

# cpp-mediator
project(mediator)
file(GLOB_RECURSE mediator_sources lib/cpp-mediator/include/*.cc)
add_library(mediator ${mediator_sources})
target_include_directories(mediator INTERFACE lib/cpp-mediator/include/)
set_target_properties(mediator PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/cpp-mediator/include/)

# vectornav c
add_subdirectory(lib/vectornav-firmware-1.1/c/ libvnc)
include_directories(lib/vectornav-firmware-1.1/c/include/)

# googletest
include_directories(lib/googletest/googletest/include/)

# vds src
include_directories(src/)
file(GLOB_RECURSE vds_sources src/*.cc)

# vds
add_executable(vds ${vds_sources})
target_link_libraries(vds LINK_PUBLIC libvnc)
target_link_libraries(vds LINK_PUBLIC mediator)
target_link_libraries(vds LINK_PUBLIC bbc)
target_link_libraries(vds LINK_PUBLIC pthread)

# tests (with coveralls)
SET(COVERAGE OFF CACHE BOOL "Coverage")
include(googletest.cmake)
set_target_properties(tests PROPERTIES LINKER_LANGUAGE CXX)
add_executable(tests tests/unit_tests.cc)
find_package (Threads)
target_link_libraries(tests gtest ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(tests PRIVATE -g -Wall -Werror -Wextra -Wpedantic -Wconversion -Wswitch-default -Wswitch-enum -Wunreachable-code -Wwrite-strings -Wcast-align -Wshadow -Wundef)
if (COVERAGE)
    target_compile_options(tests PRIVATE --coverage)
    target_link_libraries(tests PRIVATE --coverage)
endif()

