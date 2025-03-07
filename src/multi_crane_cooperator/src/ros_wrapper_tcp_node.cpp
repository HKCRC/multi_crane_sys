#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "multi_crane_msg/msg/multi_crane_msg.hpp"
#include "multi_crane_msg/msg/luffing_jib_crane_msg.hpp"
#include "multi_crane_msg/msg/tower_crane_msg.hpp"

#include "multi_crane_sys/collision_detection.hpp"
#include "multi_crane_sys/crane_utility.h"
#include "anti_collision_client.h"
#include "modbus_sensor_client.h"

void convertCraneMsg(const std::vector<CraneConfig>& crane_list , multi_crane_msg::msg::MultiCraneMsg &msg)
{
    int id = 0;
    for(auto & config : crane_list)
    {
        id++;
        if(config.type == 0)
        {
            auto lj_msg = multi_crane_msg::msg::LuffingJibCraneMsg();
            lj_msg.crane_id = id;
            lj_msg.crane_type = "luffingJibCrane";
            lj_msg.crane_x = config.x;
            lj_msg.crane_y = config.y;
            lj_msg.crane_z = config.h;
            lj_msg.boom_length = config.jib_length;
            lj_msg.boom_hor_angle = config.slewing_angle / 180 * M_PI;
            lj_msg.boom_ver_angle = config.trolley_radius_jib_angle / 180 * M_PI;
            lj_msg.hook_height = config.hoisting_height;
            msg.luffing_jib_crane_msgs.push_back(lj_msg);
        }
        else
        {
            auto tc_msg = multi_crane_msg::msg::TowerCraneMsg();
            tc_msg.crane_id = id;
            tc_msg.crane_type = "towerCrane";
            tc_msg.crane_x = config.x;
            tc_msg.crane_y = config.y;
            tc_msg.crane_z = config.h;
            tc_msg.boom_length = config.jib_length;
            tc_msg.boom_angle = config.slewing_angle / 180 * M_PI;
            tc_msg.trolley_radius = config.trolley_radius_jib_angle;
            tc_msg.hook_height = config.hoisting_height;
            msg.tower_crane_msgs.push_back(tc_msg);
        }
    }
}

int main(int argc, char ** argv)
{
    // load config file and init multiple crane system
    std::string paramFilePath = (std::string) argv[1];
    if (argc < 2) 
    {
            std::cerr << "Usage: " << argv[0] << " <path_to_param_file>\n";
            return 1;
    }

    CraneAntiCollision crane_collision;
    if(crane_collision.loadConfigFile(paramFilePath) == false)
    {
        return 1;
    }
    if(crane_collision.crane_list_.size() == 0)
    {
        std::cout<<"No crane is loaded"<<std::endl;
        return 1;
    }
    
    /*********  init tcp  ********/
    // Create and connect client
    AntiCollisionClient anti_client("127.0.0.1", 1122);
    ModbusSensorClient modbus_client("127.0.0.1", 8080);

    if (!anti_client.connect()) 
    {
        std::cerr << "Failed to connect to anti collision server" << std::endl;
        return 1;
    }
    std::cout<<"Connected to anti collision server"<<std::endl;
    if (!modbus_client.connect()) 
    {
        std::cerr << "Failed to connect to modbus server" << std::endl;
        return 1;
    }
    std::cout<<"Connected to modbus server"<<std::endl;
    anti_client.subscribe_data();
    modbus_client.subscribe_data();

    CraneGroupStatus group_sensor_data;
    SensorData modbus_sensor_data;
    CraneJointState crane_state[4];

    /******** ROS initialization *********/
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test");
  
    rclcpp::Publisher<multi_crane_msg::msg::MultiCraneMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneMsg>("multi_crane", 10);
    
    long int cnt = 0;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // Obtain crane state
        group_sensor_data = anti_client.get_data();
        modbus_sensor_data = modbus_client.get_data();
        crane_state[1].slewing_angle = modbus_sensor_data.slew_angle.value;
        crane_state[1].jib_angle = modbus_sensor_data.luff_angle.value;
        crane_state[1].hoisting_height = modbus_sensor_data.hoist_cable_length.value;
        crane_state[0].slewing_angle = group_sensor_data.encoder1_data.value;
        crane_state[0].jib_angle = group_sensor_data.imu1_data.roll;
        crane_state[0].hoisting_height = 100.0;
        crane_state[3].slewing_angle = group_sensor_data.encoder2_data.value;
        crane_state[3].jib_angle = group_sensor_data.imu2_data.roll;
        crane_state[3].hoisting_height = 100.0;

        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
        std::cout<<"Crane 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_angle: "<< crane_state[0].jib_angle << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
        std::cout<<"Crane 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_angle: "<< crane_state[1].jib_angle << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
        std::cout<<"Crane 4: slewing_angle: "<< crane_state[3].slewing_angle << ", jib_angle: "<< crane_state[3].jib_angle << ", hoisting_height: "<< crane_state[3].hoisting_height << std::endl;
        // update cranes' state
        crane_collision.updateCraneState(0, crane_state[0].slewing_angle, crane_state[0].jib_angle, crane_state[0].hoisting_height);
        crane_collision.updateCraneState(1, crane_state[1].slewing_angle, crane_state[1].jib_angle, crane_state[1].hoisting_height);
        crane_collision.updateCraneState(2, crane_state[3].slewing_angle, crane_state[3].jib_angle, crane_state[3].hoisting_height);
        
        std::cout<<"distance between cranes: "<<std::endl;
        crane_collision.showDistanceAll();

        std::cout<<"collision status: "<<std::endl;
        crane_collision.checkCollisionAll(5.0, true);

        multi_crane_msg::msg::MultiCraneMsg msg;
        convertCraneMsg(crane_collision.crane_list_, msg);
        publisher->publish(msg);

        usleep(100*1000); // microseconds
    }

}
