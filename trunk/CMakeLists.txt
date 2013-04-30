#
# Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of PROFACTOR GmbH nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL PROFACTOR GmbH BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
#
#
# This CMake file generates the necessaary test and example projects to build the
# ImageBabble library.
#
# You need to have the following requirements installed
#  ZMQ 3.2.x or greater
#  OpenCV 2.x or greater
#  Boost 1.50.x or greater
#
# Besides you will need a compiler that supports parts of C++11
#  GCC 4.2 or greater
#  MSVC 10 or greater
#
# 1 Configure
# 2 Manually point CMake to the ZMQ include directory and the specific library to link
# 3 Manually point CMake to the OpenCV build directory
# 4 Configure
# 5 Generate
# 6 Build
#

cmake_minimum_required (VERSION 2.6)

project(ImageBabble)

# Common settings
set(IMAGEBABBLE_VERSION_MAJOR "0")
set(IMAGEBABBLE_VERSION_MINOR "1")
set(IMAGEBABBLE_VERSION_BUILD "1")
set(IMAGEBABBLE_VERSION "${IMAGEBABBLE_VERSION_MAJOR}.${IMAGEBABBLE_VERSION_MINOR}.${IMAGEBABBLE_VERSION_BUILD}")
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "ImageBabble")
set(CPACK_PACKAGE_VENDOR "PROFACTOR GmbH")
set(CPACK_PACKAGE_VERSION ${IMAGEBABBLE_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${IMAGEBABBLE_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${IMAGEBABBLE_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${IMAGEBABBLE_VERSION_BUILD})
set(CPACK_SYSTEM_NAME "x86_x64")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
set(OpenCV_SHARED ON)

find_package(ZeroMQ REQUIRED)
find_package(OpenCV)
find_package(Boost COMPONENTS thread system unit_test_framework date_time chrono)
find_package(Doxygen)

# Library
add_library(lib_imagebabble STATIC 
            inc/imagebabble/core.hpp
            inc/imagebabble/fast.hpp
            inc/imagebabble/reliable.hpp
            inc/imagebabble/image_support.hpp
            inc/imagebabble/conversion/opencv.hpp
            inc/imagebabble/imagebabble.hpp)

set_target_properties(lib_imagebabble PROPERTIES LINKER_LANGUAGE CXX)

include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${ZeroMQ_INCLUDE_DIR})

# Examples
if (OpenCV_FOUND)
  set(EXAMPLE_LIBS ${ZeroMQ_LIBRARY} ${OpenCV_LIBS})
  
  add_executable(example_webcamserver examples/webcam_server.cpp)
  add_executable(example_webcamclient examples/webcam_client.cpp)  
  add_executable(example_imagegroupserver examples/imagegroup_server.cpp)
  add_executable(example_imagegroupclient examples/imagegroup_client.cpp)  
  add_executable(example_customdata examples/customdata_server_client.cpp)  

  target_link_libraries(example_webcamserver ${EXAMPLE_LIBS})
  target_link_libraries(example_webcamclient ${EXAMPLE_LIBS})
  target_link_libraries(example_imagegroupserver ${EXAMPLE_LIBS})
  target_link_libraries(example_imagegroupclient ${EXAMPLE_LIBS})
  target_link_libraries(example_customdata ${EXAMPLE_LIBS})
endif()

# Tests
if (Boost_FOUND AND OpenCV_FOUND)
  set(TEST_LIBS ${ZeroMQ_LIBRARY} ${Boost_LIBRARIES} ${OpenCV_LIBS})
  include_directories(${Boost_INCLUDE_DIR})
  add_definitions(-DBOOST_ALL_DYN_LINK)
  
  add_executable(
    test_imagebabble 
    tests/main.cpp
    tests/test_reliable.cpp
    tests/test_fast.cpp
    tests/test_data_types.cpp
    tests/test_image_support.cpp
    tests/test_image_opencv.cpp)

  target_link_libraries(test_imagebabble pgm ${TEST_LIBS})
endif()


# Documentation
if (DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(BuildDocumentation ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
endif()

# Installation
install(DIRECTORY inc/ DESTINATION inc FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY examples/ DESTINATION examples FILES_MATCHING PATTERN "*.cpp")
install(DIRECTORY ${PROJECT_BINARY_DIR}/Documentation/html/ DESTINATION doc)

# Packaging
include(CPack)
