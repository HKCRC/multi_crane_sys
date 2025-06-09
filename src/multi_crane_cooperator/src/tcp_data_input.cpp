#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "multi_crane_msg/msg/crane_joint_state_msg.hpp"
#include "multi_crane_msg/msg/multi_crane_joint_state_msg.hpp"

#include "anti_collision_client.h"
#include "modbus_sensor_client.h"
#include "multi_crane_sys/crane_utility.h"

#define SHOW_DEBUG_INFO true
#define CRANE_NUMBER 3

void convertCraneJointStateMsg(CraneJointState crane_state[], multi_crane_msg::msg::MultiCraneJointStateMsg &multi_msg)
{
    for(int i=0; i < CRANE_NUMBER; i++)
    {
        auto msg = multi_crane_msg::msg::CraneJointStateMsg();
        msg.slewing_angle = crane_state[i].slewing_angle;
        msg.jib_trolley = crane_state[i].jib_trolley;
        msg.hoisting_length = crane_state[i].hoisting_height;
        multi_msg.crane_joint_state.push_back(msg);
    }
}

void convertSensorDataToCraneJointState(const SensorData &modbus_sensor_data, const CraneGroupStatus &neighbor_data, CraneJointState crane_state[])
{
    //main crane
    double slewing_coeffs[2] = {-42.2958, 87047.3787};
    double luffing_coeffs[4] = {8.83387468e-07, -5.22125503e-03, 1.05831331e+01, -7.30710612e+03};
    double hoisting_coeffs[2] = {-0.7120, 1464.8306};
    double slewing_position_encoder = modbus_sensor_data.slew_angle.value;
    double luffing_position_encoder = modbus_sensor_data.luff_angle.value;
    double hoisting_position_encoder = modbus_sensor_data.hoist_cable_length.value;
    
    double tc2_slew_angle = fmod(slewing_coeffs[0] * slewing_position_encoder + slewing_coeffs[1], 360.0);
    if (tc2_slew_angle < 0) {
        tc2_slew_angle += 360.0;
    }
    crane_state[1].slewing_angle = tc2_slew_angle;
    crane_state[1].jib_trolley = luffing_coeffs[0] * pow(luffing_position_encoder,3) + luffing_coeffs[1] * pow(luffing_position_encoder, 2) + luffing_coeffs[2] * luffing_position_encoder + luffing_coeffs[3];
    crane_state[1].hoisting_height = hoisting_coeffs[0] * hoisting_position_encoder + hoisting_coeffs[1];

    //neighbor cranes
    double tc1_slew_angle = fmod((1829.7822 - neighbor_data.encoder1_data.value) * 360.0/8.5 + 90.0,360.0); //deg
    double tc4_slew_angle = fmod((2046.7278 - neighbor_data.encoder2_data.value) * 360.0/8.5 + 90.0,360.0);
    double tc1_luffing_angle = -neighbor_data.imu1_data.pitch; //deg
    double tc4_luffing_angle = -neighbor_data.imu2_data.pitch; 

    if (tc1_slew_angle < 0) {
        tc1_slew_angle += 360.0;
    }
    if (tc4_slew_angle < 0) {
        tc4_slew_angle += 360.0;
    }
    crane_state[0].slewing_angle = tc1_slew_angle;
    crane_state[0].jib_trolley = tc1_luffing_angle;
    crane_state[0].hoisting_height = 100.0;
    crane_state[2].slewing_angle = tc4_slew_angle;
    crane_state[2].jib_trolley = tc4_luffing_angle;
    crane_state[2].hoisting_height = 100.0;
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
    // connect to server to get crane data 
    AntiCollisionClient anti_client("192.168.1.3", 54321); // neighbor cranes
    ModbusSensorClient modbus_client("192.168.1.2", 8080); // main crane

    std::cout << "Connecting to crane server..." << std::endl;

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
    std::cout<<"Connected to modbus server!"<<std::endl;
    anti_client.subscribe_data();
    modbus_client.subscribe_data();

    CraneGroupStatus neighbor_sensor_data;
    SensorData modbus_sensor_data;
    CraneJointState crane_state[CRANE_NUMBER];

    /******** ROS initialization *********/
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("data_input_tcp");
  
    rclcpp::Publisher<multi_crane_msg::msg::MultiCraneJointStateMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneJointStateMsg>("multi_crane_input", 10);
    
    signal(SIGINT, signalHandler); // 捕获异常SIGINT
    
    long int cnt = 0;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        // Obtain sensor data and convert to crane state
        neighbor_sensor_data = anti_client.get_data();
        modbus_sensor_data = modbus_client.get_data();
        convertSensorDataToCraneJointState(modbus_sensor_data, neighbor_sensor_data, crane_state);

        if(SHOW_DEBUG_INFO)
        {
            std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
            std::cout<<"PTC 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_trolley: "<< crane_state[0].jib_trolley << ", hoisting_length: "<< crane_state[0].hoisting_height << std::endl;
            std::cout<<"PTC 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_trolley: "<< crane_state[1].jib_trolley << ", hoisting_length: "<< crane_state[1].hoisting_height << std::endl;
            std::cout<<"PTC 4: slewing_angle: "<< crane_state[2].slewing_angle << ", jib_trolley: "<< crane_state[2].jib_trolley << ", hoisting_length: "<< crane_state[2].hoisting_height << std::endl;
            std::cout<<"Encoder timestamp, tc1: "<< fmod(neighbor_sensor_data.encoder1_data.timestamp, 1e6) <<", tc4: "<< fmod(neighbor_sensor_data.encoder2_data.timestamp, 1e6) << std::endl;
            std::cout<<"IMU timestamp, tc1: "<< fmod(neighbor_sensor_data.imu1_data.timestamp, 1e6)<<", tc4: "<< fmod(neighbor_sensor_data.imu2_data.timestamp, 1e6) << std::endl;
        }

        multi_crane_msg::msg::MultiCraneJointStateMsg msg;
        convertCraneJointStateMsg(crane_state, msg);
        publisher->publish(msg);

        usleep(100*1000); // microseconds
    }

    rclcpp::shutdown();
    return 0;
}