#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
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

#include "multi_crane_cooperator/crane_utility.h"
#include "multi_crane_cooperator/collision_detection.hpp"

using namespace std;

// init crane class by loading configuration file
std::vector<LuffingJibCraneConfig> luffing_crane_list;
std::vector<TowerCraneConfig> tower_crane_list;

// 保存终端原始设置
termios originalTty;

// 设置非阻塞输入
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
  tcsetattr(STDIN_FILENO, TCSANOW, &originalTty); // 恢复原始设置
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
            luffing_crane_list[0].slewing_angle += 5.0f;
            break;
        case 'd':
            luffing_crane_list[0].slewing_angle -= 5.0f;
            break;
        case 'w': 
            luffing_crane_list[0].jib_angle += 5.0f;
            break;
        case 's':
            luffing_crane_list[0].jib_angle -= 5.0f;
            break;
        case 'i': 
            luffing_crane_list[0].hoisting_height -= 1.0f;
            break;
        case 'k':
            luffing_crane_list[0].hoisting_height += 1.0f;
            break;
        case '4':
            tower_crane_list[0].slewing_angle += 5.0f;
            break;
        case '6':
            tower_crane_list[0].slewing_angle -= 5.0f;
            break;
        case '8': 
            tower_crane_list[0].trolley_radius += 1.0f;
            break;
        case '2':
            tower_crane_list[0].trolley_radius -= 1.0f;
            break;
        case '-': 
            tower_crane_list[0].hoisting_height -= 1.0f;
            break;
        case '+':
            tower_crane_list[0].hoisting_height += 1.0f;
            break;
        default:
            std::cout << "Invalid input" << std::endl;
            break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool loadConfigFile(std::string file_name)
{
  YAML::Node config = YAML::LoadFile(file_name);

  if(!config)
  {
    std::cout<<"Open config file: "<<file_name<<" failed"<<std::endl;
    return false;
  }
  
  if(config["luffing_cranes"])
  {
    LuffingJibCraneConfig ljc;
    for(const auto& crane : config["luffing_cranes"])
    {
      ljc.x = crane["parameters"]["x"].as<double>();
      ljc.y = crane["parameters"]["y"].as<double>();
      ljc.h = crane["parameters"]["h"].as<double>();
      ljc.d = crane["parameters"]["d"].as<double>();
      ljc.jib_length = crane["parameters"]["jib_length"].as<double>();
      ljc.slewing_angle = 0;
      ljc.jib_angle = 0;
      ljc.hoisting_height = ljc.h;

      luffing_crane_list.push_back(ljc);
    }
  }

  if(config["tower_cranes"])
  {
    TowerCraneConfig tc;
    for(const auto& crane : config["tower_cranes"])
    {
      tc.x = crane["parameters"]["x"].as<double>();
      tc.y = crane["parameters"]["y"].as<double>();
      tc.h = crane["parameters"]["h"].as<double>();
      tc.jib_length = crane["parameters"]["jib_length"].as<double>();
      tc.slewing_angle = 0;
      tc.trolley_radius = tc.jib_length;
      tc.hoisting_height = tc.h;

      tower_crane_list.push_back(tc);
    }
  }

  return true;
}

void pushLuffingJibCraneMsg(const LuffingJibCraneConfig& config, int craneID, multi_crane_msg::msg::MultiCraneMsg &msg)
{
  auto lj_msg = multi_crane_msg::msg::LuffingJibCraneMsg();
  lj_msg.crane_id = craneID;
  lj_msg.crane_type = "luffingJibCrane";
  lj_msg.crane_x = config.x;
  lj_msg.crane_y = config.y;
  lj_msg.crane_z = config.h;
  lj_msg.boom_length = config.jib_length;
  lj_msg.boom_hor_angle = config.slewing_angle / 180 * M_PI;
  lj_msg.boom_ver_angle = config.jib_angle / 180 * M_PI;
  lj_msg.hook_height = config.hoisting_height;
  msg.luffing_jib_crane_msgs.push_back(lj_msg);
}
void pushTowerCraneMsg(const TowerCraneConfig& config, int craneID, multi_crane_msg::msg::MultiCraneMsg &msg)
{
  auto tc_msg = multi_crane_msg::msg::TowerCraneMsg();
  tc_msg.crane_id = craneID;
  tc_msg.crane_type = "towerCrane";
  tc_msg.crane_x = config.x;
  tc_msg.crane_y = config.y;
  tc_msg.crane_z = config.h;
  tc_msg.boom_length = config.jib_length;
  tc_msg.boom_angle = config.slewing_angle / 180 * M_PI;
  tc_msg.trolley_radius = config.trolley_radius;
  tc_msg.hook_height = config.hoisting_height;
  msg.tower_crane_msgs.push_back(tc_msg);
}

int main(int argc, char ** argv)
{
  if (argc < 2) 
  {
        std::cerr << "Usage: " << argv[0] << " <path_to_param_file>\n";
        return 1;
  }

  std::string paramFilePath = (std::string) argv[1];

  if(loadConfigFile(paramFilePath) == false)
  {
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test");

  rclcpp::Publisher<multi_crane_msg::msg::MultiCraneMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneMsg>("multi_crane", 10);

  std::unique_ptr<std::thread> thread_input = std::make_unique<std::thread>(&keyboardListener); // using keyboard to control crane status

  long int cnt = 0;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    // update the joint state of cranes

    // call check_collision function
    for(int i = 0; i < luffing_crane_list.size(); i++)
      for(int j = i+1; j < luffing_crane_list.size(); j++)
      {
        if(checkCollisionBetweenMixCranes(luffing_crane_list[i], tower_crane_list[j], 4.0))
        {
          std::cout<<"Warnning: "<< cnt++ << ", CRANE "<< i+1 << " & " << "CRANE " << j+1 <<std::endl;
        }
        else
          cnt = 0;
      }
      

    // send data to UI to show animation
    auto msg = multi_crane_msg::msg::MultiCraneMsg();
    for(int i = 0; i < luffing_crane_list.size(); i++)
    {
      pushLuffingJibCraneMsg(luffing_crane_list[i], i+1, msg);
    }

    publisher->publish(msg);
    

    std::this_thread::sleep_for(100ms);
  }
  
  restoreTerminalSettings();

  rclcpp::shutdown();

  return 0;
}
