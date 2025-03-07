cmake_minimum_required(VERSION 3.5)
project(cpp_multi_crane_sys)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(multi_crane_sys src/multi_crane_sys/collision_detection.cpp)
target_link_libraries(multi_crane_sys yaml-cpp)
target_include_directories(multi_crane_sys PUBLIC
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

add_library(tcp_multi_crane INTERFACE)
target_link_libraries(tcp_multi_crane INTERFACE pthread)
target_include_directories(tcp_multi_crane INTERFACE
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/dependence/modbus_tcp_devices>
  $<INSTALL_INTERFACE:include>
)

add_subdirectory(test)