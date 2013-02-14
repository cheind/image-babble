
# Note, OpenCV has no direct FindModule, but requires you to set OpenCV dir to the build subdirectory of the extracted OpenCV.

cmake_minimum_required (VERSION 2.6)

project(ImageBabble)

# Allow ZMQ Finder to work
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(ZeroMQ REQUIRED)

set(OpenCV_SHARED ON)
find_package(OpenCV)

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS thread system unit_test_framework date_time chrono)


include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${ZeroMQ_INCLUDE_DIR})

# Examples

if(OpenCV_FOUND)
  set(EXAMPLE_LIBS ${ZeroMQ_LIBRARY} ${OpenCV_LIBS})
  
  add_executable(example_webcamserver examples/webcam_server.cpp)
  add_executable(example_webcamclient examples/webcam_client.cpp)  

  target_link_libraries(example_webcamserver ${EXAMPLE_LIBS})
  target_link_libraries(example_webcamclient ${EXAMPLE_LIBS})
endif()

# Tests
if(Boost_FOUND)
  set(TEST_LIBS ${ZeroMQ_LIBRARY} ${Boost_LIBRARIES})
  include_directories(${Boost_INCLUDE_DIR})
  add_definitions(-DBOOST_ALL_DYN_LINK)
  
  add_executable(
    test_imagebabble 
    tests/main.cpp
    tests/test_reliable.cpp)

  target_link_libraries(test_imagebabble ${TEST_LIBS})

endif()









