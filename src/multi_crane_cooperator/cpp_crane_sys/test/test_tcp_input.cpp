#include <cstdio>
#include <unistd.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <termios.h>
#include <yaml-cpp/yaml.h>

#include "multi_crane_sys/collision_detection.hpp"
#include "multi_crane_sys/crane_utility.h"
#include "anti_collision_client.h"
#include "modbus_sensor_client.h"

using namespace std;

CraneJointState crane_state[4];

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

    long unsigned int cnt = 0;
    while(true)
    {
        // anti_client.print_status();
        // modbus_client.print_status();
        group_sensor_data = anti_client.get_data();
        modbus_sensor_data = modbus_client.get_data();
        // Update crane state
        crane_state[1].slewing_angle = modbus_sensor_data.slew_angle.value;
        crane_state[1].jib_angle = modbus_sensor_data.luff_angle.value;
        crane_state[1].hoisting_height = modbus_sensor_data.hoist_cable_length.value;
        crane_state[i].current_time  = current_time;

        crane_state[0].slewing_angle = group_sensor_data.encoder1_data.value;
        crane_state[0].jib_angle = group_sensor_data.imu1_data.roll;
        crane_state[0].hoisting_height = 100.0;
        crane_state[i].current_time  = current_time;
        
        crane_state[3].slewing_angle = group_sensor_data.encoder2_data.value;
        crane_state[3].jib_angle = group_sensor_data.imu2_data.roll;
        crane_state[3].hoisting_height = 100.0;
        crane_state[i].current_time  = current_time;

        
        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
        std::cout<<"Crane 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_angle: "<< crane_state[0].jib_angle << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
        std::cout<<"Crane 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_angle: "<< crane_state[1].jib_angle << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
        std::cout<<"Crane 4: slewing_angle: "<< crane_state[3].slewing_angle << ", jib_angle: "<< crane_state[3].jib_angle << ", hoisting_height: "<< crane_state[3].hoisting_height << std::endl;

        // update cranes' state and slewing velocity
        crane_collision.updateAllCraneState(crane_state);
        crane_collision.updateCraneSlewingVelocity();
        
        std::cout<<"distance between cranes: "<<std::endl;
        crane_collision.showDistanceAll();

        std::cout<<"collision status: "<<std::endl;
        crane_collision.checkCollisionAll(5.0, true);

        std::cout<<"predict collision status: "<<std::endl;
        crane_collision.predictCollisionAll(5.0, true);
        // crane_collision.predictCollisionMainCraneNeighbor(5.0, true);

        usleep(100*1000); // microseconds
    }
}