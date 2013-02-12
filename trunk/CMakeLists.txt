cmake_minimum_required (VERSION 2.6)

project(ImageBabble)

# Allow ZMQ Finder to work
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(ZeroMQ REQUIRED)


include_directories(${ZeroMQ_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/external)

add_executable(Example examples/simple_server.cpp)

set(LIBS ${LIBS} ${ZeroMQ_LIBRARIES})
target_link_libraries(Example ${LIBS})



