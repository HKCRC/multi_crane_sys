cmake_minimum_required(VERSION 3.5)
project(multi_crane_cooperator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(multi_crane_msg REQUIRED)

add_subdirectory(cpp_crane_sys)

add_executable(tcp_data_input src/tcp_data_input.cpp)
ament_target_dependencies(tcp_data_input rclcpp std_msgs multi_crane_msg)
target_link_libraries(tcp_data_input multi_crane_sys)

add_executable(keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(keyboard_input rclcpp std_msgs multi_crane_msg)
target_link_libraries(keyboard_input multi_crane_sys)

add_executable(bt_collision_detection src/bt_collision_detection.cpp)
ament_target_dependencies(bt_collision_detection rclcpp std_msgs multi_crane_msg)
target_link_libraries(bt_collision_detection multi_crane_sys)

add_executable(ros_wrapper_tcp_node test/ros_wrapper_tcp_node.cpp)
ament_target_dependencies(ros_wrapper_tcp_node rclcpp std_msgs multi_crane_msg)
target_link_libraries(ros_wrapper_tcp_node multi_crane_sys)

install(TARGETS ros_wrapper_tcp_node
  tcp_data_input
  keyboard_input
  bt_collision_detection
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
