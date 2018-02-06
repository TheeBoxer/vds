cmake_minimum_required(VERSION 2.8)
project(vds)

project(bbc)
file(GLOB_RECURSE bbc_sources lib/beaglebone-c/src/*.cc)
add_library(bbc ${bbc_sources})
target_include_directories(bbc INTERFACE lib/beaglebone-c/include/)
set_target_properties(bbc PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/beaglebone-c/include/)

project(mediator)
file(GLOB_RECURSE mediator_sources lib/cpp-mediator/include/*.cc)
add_library(mediator ${mediator_sources})
target_include_directories(mediator INTERFACE lib/cpp-mediator/include/)
set_target_properties(mediator PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/cpp-mediator/include/)


add_subdirectory(lib/vectornav-firmware-1.1/c/ libvnc)
include_directories(lib/vectornav-firmware-1.1/c/include/)

include_directories(lib/googletest/googletest/include/)

include_directories(src/)
file(GLOB_RECURSE vds_sources src/*.cc)
add_executable(vds ${vds_sources})
target_link_libraries(vds LINK_PUBLIC libvnc)
target_link_libraries(vds LINK_PUBLIC mediator)
target_link_libraries(vds LINK_PUBLIC bbc)

if (UNIX OR APPLE)
	target_link_libraries(vds LINK_PUBLIC pthread)
endif()
