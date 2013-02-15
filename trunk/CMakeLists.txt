
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

find_package(Doxygen)


add_library(lib_imagebabble STATIC 
            inc/imagebabble/core.hpp
            inc/imagebabble/image_support.hpp
            inc/imagebabble/imagebabble.hpp)
set_target_properties(lib_imagebabble PROPERTIES LINKER_LANGUAGE CXX)


include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${ZeroMQ_INCLUDE_DIR})

# Examples

if (OpenCV_FOUND)
  set(EXAMPLE_LIBS ${ZeroMQ_LIBRARY} ${OpenCV_LIBS})
  
  add_executable(example_webcamserver examples/webcam_server.cpp)
  add_executable(example_webcamclient examples/webcam_client.cpp)  

  target_link_libraries(example_webcamserver ${EXAMPLE_LIBS})
  target_link_libraries(example_webcamclient ${EXAMPLE_LIBS})
endif()

# Tests
if (Boost_FOUND)
  set(TEST_LIBS ${ZeroMQ_LIBRARY} ${Boost_LIBRARIES})
  include_directories(${Boost_INCLUDE_DIR})
  add_definitions(-DBOOST_ALL_DYN_LINK)
  
  add_executable(
    test_imagebabble 
    tests/main.cpp
    tests/test_reliable.cpp
    tests/test_data_types.cpp
    tests/test_image_support.cpp)

  target_link_libraries(test_imagebabble ${TEST_LIBS})

endif()

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(BuildDocumentation ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
endif()





