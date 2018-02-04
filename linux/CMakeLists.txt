cmake_minimum_required(VERSION 2.8)

project(vds)

project(mediator CXX)
file(GLOB_RECURSE mediator_sources lib/cpp-mediator/include/*.hpp)
add_library(mediator ${mediator_sources})
target_include_directories(mediator INTERFACE lib/cpp-mediator/include/)
set_target_properties(mediator PROPERTIES LINKER_LANGUAGE CXX)


include_directories(lib/cpp-mediator/include/)
include_directories(lib/BlackLib/v3_0/)
include_directories(lib/vectornav-firmware-1.1/c/include/)
include_directories(lib/googletest/googletest/include/)
include_directories(src/)
file(GLOB_RECURSE vds_sources src/*.cc)
add_executable(vds ${vds_sources})
target_link_libraries(mediator)
