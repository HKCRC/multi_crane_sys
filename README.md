# multi_crane_sys
A ROS2 workspace to implement and illustrate the collision detection between luffing jib crane and tower crane system.

## Requirements
- ROS2
- yaml-cpp

```bash
$ sudo apt update
$ sudo apt install libyaml-cpp-dev
```
## Package explanation
- visual_collision \
To plot cranes diagram

- multi_crane_cooperator \
To initiate the configuration of cranes and check the collision between them, as well as send crane's state to UI. It supports the config file loading from yaml file, which is placed in "config" folder.

## Running
- Download
```bash
$ git clone https://github.com/HKCRC/multi_crane_sys.git
```
- Build
```bash
$ cd pathTo/multi_crane_sys
$ colcon build
$ source install/setup.bash
$ ros2 run multi_crane_cooperator test_collision_check ./multi_crane_cooperator/config/crane_setting.yaml
```
You can change crane's joint states using keyboard. Please refer to the code for the detail. 

- Open a terminal to run UI for display cranes' diagram
```bash
$ cd pathTo/multi_crane_sys
$ source install/setup.bash
$ ros2 run visual_collision visual_collision
```

