#include <iostream>
#include <string>
#include <thread>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "multi_crane_msg/msg/crane_joint_state_msg.hpp"
#include "multi_crane_msg/msg/multi_crane_joint_state_msg.hpp"

#include "anti_collision_client.h"
#include "modbus_sensor_client.h"
#include "multi_crane_sys/crane_utility.h"

#define SHOW_DEBUG_INFO true
#define CRANE_NUMBER 3


CraneJointState crane_state[CRANE_NUMBER];

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

termios originalTty;
void setNonBlockingInput()
{
  termios tty;
  tcgetattr(STDIN_FILENO, &tty); // 获取当前终端设置
  originalTty = tty; // 保存原始设置
  tty.c_lflag &= ~ICANON; // 禁用规范模式
  tty.c_lflag &= ~ECHO;   // 禁用回显
  tcsetattr(STDIN_FILENO, TCSANOW, &tty); // 应用设置
}

void restoreTerminalSettings() 
{
  tcsetattr(STDIN_FILENO, TCSANOW, &originalTty); // recover settings
}
void keyboardListener() 
{
  setNonBlockingInput();
  char input;
  std::cout<< "go into thread"<<std::endl;
  while (true) 
  {
    std::cin >> input;

    switch (input) {
        case 'a':
            crane_state[0].slewing_angle += 5.0f;
            break;
        case 'd':
            crane_state[0].slewing_angle -= 5.0f;
            break;
        case 'w': 
            crane_state[0].jib_trolley += 5.0f;
            break;
        case 's':
            crane_state[0].jib_trolley -= 5.0f;
            break;
        case 'r': 
            crane_state[0].hoisting_height -= 1.0f;
            break;
        case 'f':
            crane_state[0].hoisting_height += 1.0f;
            break;
        case 'h':
            crane_state[1].slewing_angle += 5.0f;
            break;
        case 'k':
            crane_state[1].slewing_angle -= 5.0f;
            break;
        case 'u': 
            crane_state[1].jib_trolley += 5.0f;
            break;
        case 'j':
            crane_state[1].jib_trolley -= 5.0f;
            break;
        case 'o': 
            crane_state[1].hoisting_height -= 1.0f;
            break;
        case 'l':
            crane_state[1].hoisting_height += 1.0f;
            break;
        case '4':
            crane_state[2].slewing_angle += 5.0f;
            break;
        case '6':
            crane_state[2].slewing_angle -= 5.0f;
            break;
        case '8': 
            crane_state[2].jib_trolley += 5.0f;
            break;
        case '2':
            crane_state[2].jib_trolley -= 5.0f;
            break;
        case '-': 
            crane_state[2].hoisting_height -= 1.0f;
            break;
        case '+':
            crane_state[2].hoisting_height += 1.0f;
            break;
        default:
            std::cout << "Invalid input" << std::endl;
            break;
    }
    usleep(10*1000);
  }
  restoreTerminalSettings(); // 确保退出时恢复设置
}

void signalHandler(int signal) 
{
    std::cout << "Caught signal: " << signal << std::endl;
    restoreTerminalSettings();
    rclcpp::shutdown();
    std::cout << "Exiting program..." << std::endl;
    exit(0);
}

int main(int argc, char ** argv)
{
    /******** ROS initialization *********/
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("data_input_tcp");
  
    rclcpp::Publisher<multi_crane_msg::msg::MultiCraneJointStateMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneJointStateMsg>("multi_crane_input", 10);
    
    std::unique_ptr<std::thread> thread_input = std::make_unique<std::thread>(&keyboardListener); // using keyboard to control crane status

    signal(SIGINT, signalHandler); // 捕获异常SIGINT

    long int cnt = 0;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if(SHOW_DEBUG_INFO)
        {
            std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
            std::cout<<"PTC 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_trolley: "<< crane_state[0].jib_trolley << ", hoisting_length: "<< crane_state[0].hoisting_height << std::endl;
            std::cout<<"PTC 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_trolley: "<< crane_state[1].jib_trolley << ", hoisting_length: "<< crane_state[1].hoisting_height << std::endl;
            std::cout<<"PTC 4: slewing_angle: "<< crane_state[2].slewing_angle << ", jib_trolley: "<< crane_state[2].jib_trolley << ", hoisting_length: "<< crane_state[2].hoisting_height << std::endl;
        }

        multi_crane_msg::msg::MultiCraneJointStateMsg msg;
        convertCraneJointStateMsg(crane_state, msg);
        publisher->publish(msg);

        usleep(100*1000); // microseconds
    }

    rclcpp::shutdown();
    return 0;
}