cmake_minimum_required(VERSION 2.8)
project(vds)

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
target_link_libraries(vds LINK_PUBLIC pthread)
target_compile_options(vds PRIVATE -std=c++11)
