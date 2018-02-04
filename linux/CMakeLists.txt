cmake_minimum_required(VERSION 2.8)
project(vds)

project(blacklib)
file(GLOB_RECURSE blacklib_sources lib/BlackLib/v3_0/*.cpp)
add_library(blacklib ${blacklib_sources})
target_include_directories(blacklib INTERFACE lib/BlackLib/v3_0/)
set_target_properties(blacklib PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/BlackLib/v3_0/)

project(mediator)
file(GLOB_RECURSE mediator_sources lib/cpp-mediator/include/*.cc)
add_library(mediator ${mediator_sources})
target_include_directories(mediator INTERFACE lib/cpp-mediator/include/)
set_target_properties(mediator PROPERTIES LINKER_LANGUAGE CXX)
include_directories(lib/cpp-mediator/include/)


add_subdirectory(lib/vectornav-firmware-1.1/c/ libvnc)

#project(vectornav)
#file(GLOB_RECURSE vectornav_sources lib/vectornav-firmware-1.1/c/*.c)
#add_library(vectornav ${vectornav_sources})
#target_include_directories(vectornav INTERFACE lib/vectornav-firmware-1.1/c/include/)
#set_target_properties(vectornav PROPERTIES LINKER_LANGUAGE C)
include_directories(lib/vectornav-firmware-1.1/c/include/)

include_directories(lib/googletest/googletest/include/)

include_directories(src/)
file(GLOB_RECURSE vds_sources src/*.cc)
add_executable(vds ${vds_sources})
target_link_libraries(vds LINK_PUBLIC blacklib)
target_link_libraries(vds LINK_PUBLIC libvnc)
target_link_libraries(vds LINK_PUBLIC mediator)

if (UNIX OR APPLE)
	target_link_libraries(vds LINK_PUBLIC pthread)
endif()