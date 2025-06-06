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
#include "multi_crane_msg/msg/crane_joint_state_msg.hpp"
#include "multi_crane_msg/msg/multi_crane_joint_state_msg.hpp"

#include "multi_crane_sys/collision_detection.hpp"
#include "multi_crane_sys/crane_utility.h"

#define DEBUG_FLAG true
#define THRESHOLD 10.0 // threshold for collision detection, unit: m

std::vector<CraneJointState> crane_state(3);

void convertCraneMsg(const std::vector<CraneConfig>& crane_list , multi_crane_msg::msg::MultiCraneMsg &msg)
{
    int id = 0;
    int crane_id[3]={1,2,4};
    for(auto & config : crane_list)
    {
        if(config.type == 0)
        {
            auto lj_msg = multi_crane_msg::msg::LuffingJibCraneMsg();
            lj_msg.crane_id = crane_id[id];
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
            tc_msg.crane_id = crane_id[id];
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
        id++;
    }
}

void callbackCraneState(const multi_crane_msg::msg::MultiCraneJointStateMsg::SharedPtr msg)
{
    // Update crane state from the received message
    for (size_t i = 0; i < msg->crane_joint_state.size(); ++i) 
    {
        crane_state[i].slewing_angle = msg->crane_joint_state[i].slewing_angle;
        crane_state[i].jib_trolley = msg->crane_joint_state[i].jib_trolley;
        crane_state[i].hoisting_height = 40 + sin(msg->crane_joint_state[i].jib_trolley / 180 * M_PI) * 50;
    }
}

void signalHandler(int signal) 
{
    std::cout << "Caught signal: " << signal << std::endl;
    rclcpp::shutdown();
    std::cout << "Exiting program..." << std::endl;
    exit(0);
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

    /******** ROS initialization *********/
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_Collision_detection");

    auto sub_crane_state = node->create_subscription<multi_crane_msg::msg::MultiCraneJointStateMsg>(
        "multi_crane_input", 10, 
        std::bind(callbackCraneState, std::placeholders::_1));
    
    auto publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneMsg>("multi_crane", 10);
    
    long int cnt = 0;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;

        // update cranes' state
        crane_collision.updateAllCraneState(crane_state);

        // adopt the conservative strategy to check collision
        std::cout<<"conservative collision detection status: "<<std::endl;
        u_char direction = crane_collision.checkBTMainCraneAllowedMotion(30.0, THRESHOLD);

        if(DEBUG_FLAG)
        {
            std::cout<<"TC1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_trolley: "<< crane_state[0].jib_trolley << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
            std::cout<<"TC2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_trolley: "<< crane_state[1].jib_trolley << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
            std::cout<<"TC4: slewing_angle: "<< crane_state[2].slewing_angle << ", jib_trolley: "<< crane_state[2].jib_trolley << ", hoisting_height: "<< crane_state[2].hoisting_height << std::endl;
            if(direction & ( 1<<5 ))
            {
                std::cout<<"!!! TC2's Slewing + is not allowed"<<std::endl;
            }
            if(direction & ( 1<<4 ))
            {
                std::cout<<"!!! TC2's Slewing - is not allowed"<<std::endl;
            }

            multi_crane_msg::msg::MultiCraneMsg msg;
            convertCraneMsg(crane_collision.crane_list_, msg);
            publisher->publish(msg);
        }

        usleep(100*1000); // microseconds
    }

    rclcpp::shutdown();
    return 0;
}
