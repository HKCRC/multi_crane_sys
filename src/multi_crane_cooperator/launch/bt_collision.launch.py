import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def get_package_source_directory(package_name):
    # Get the package path in shared directory
    share_path = get_package_share_directory(package_name)
    
    parent_path = os.path.dirname(share_path)
    # Find the package source directory
    while os.path.basename(parent_path) != 'install' and parent_path != '/':
        parent_path = os.path.dirname(parent_path)

    if os.path.basename(parent_path) == 'install':
        print(f"Source workspace path: {os.path.dirname(parent_path)}")
    else:
        print("Could not find 'src' directory in the parent paths.")
        
    return os.path.join(os.path.dirname(parent_path), 'src', package_name)

def generate_launch_description():
    package_name = 'multi_crane_cooperator'
    package_path = get_package_source_directory(package_name)
    param_file_path = os.path.join(package_path, 'config', 'crane_setting.yaml')
    config_file_path = os.path.join(package_path, 'config', 'crane_setting.yaml')
    
    print(f"Config file path: {param_file_path}")

    return LaunchDescription([
        Node(
            package='multi_crane_cooperator', 
            executable='bt_collision_detection',         
            name='bt_collision_detection',
            output="screen",
            arguments=[config_file_path]
            # parameters=[{"config_file_path": config_file_path}]
        ),
        Node(
            package='multi_crane_cooperator', 
            executable='tcp_data_input',         
            name='tcp_data_input',
        )
        
    ])