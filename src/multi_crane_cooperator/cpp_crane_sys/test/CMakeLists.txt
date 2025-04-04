find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multi_crane_msg REQUIRED)

add_executable(test_tcp_input ./test_tcp_input.cpp)
target_link_libraries(test_tcp_input multi_crane_sys tcp_multi_crane)

add_executable(test_keyboard_input ./test_keyboard_input.cpp)
target_link_libraries(test_keyboard_input multi_crane_sys tcp_multi_crane)

add_executable(test_keyboard_input_ros ./test_keyboard_input_ros.cpp)
target_link_libraries(test_keyboard_input_ros 
    multi_crane_sys 
    tcp_multi_crane
    ${rclcpp_LIBRARIES}
    ${multi_crane_msg_LIBRARIES}
)

ament_target_dependencies(test_keyboard_input_ros
    rclcpp
    multi_crane_msg
)