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
            luffing_crane_list[0].hoisting_height -= 5.0f;
            break;
        case 'k':
            luffing_crane_list[0].hoisting_height += 5.0f;
            break;
        case '4':
            tower_crane_list[0].slewing_angle += 5.0f;
            break;
        case '6':
            tower_crane_list[0].slewing_angle -= 5.0f;
            break;
        case '8': 
            tower_crane_list[0].trolley_radius += 5.0f;
            break;
        case '2':
            tower_crane_list[0].trolley_radius -= 5.0f;
            break;
        case '-': 
            tower_crane_list[0].hoisting_height -= 5.0f;
            break;
        case '+':
            tower_crane_list[0].hoisting_height += 5.0f;
            break;
        default:
            std::cout << "Invalid input" << std::endl;
            break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test");

  rclcpp::Publisher<multi_crane_msg::msg::MultiCraneMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneMsg>("multi_crane", 10);

  LuffingJibCraneConfig ljc = {0, 0, 40, 0, 50, 0, 0, 0};
  luffing_crane_list.push_back(ljc);

  TowerCraneConfig tc = {80, 0, 40, 50, 0, 0, 0};
  tower_crane_list.push_back(tc);

  // initiate joint status
  luffing_crane_list[0].slewing_angle = 0;
  luffing_crane_list[0].jib_angle = 0;
  luffing_crane_list[0].hoisting_height = 40;

  tower_crane_list[0].slewing_angle = 90;
  tower_crane_list[0].trolley_radius = 50;
  tower_crane_list[0].hoisting_height = 40;

  std::unique_ptr<std::thread> thread_input = std::make_unique<std::thread>(&keyboardListener); // using keyboard to control crane status


  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    // call check_collision function
    if(checkCollisionBetweenMixCranes(luffing_crane_list[0], tower_crane_list[0], 4.0))
      std::cout<<"With the risk of collision"<<std::endl;

    // send data to UI to show animation
    auto msg = multi_crane_msg::msg::MultiCraneMsg();
    auto tc_msg = multi_crane_msg::msg::TowerCraneMsg();
    auto lj_msg = multi_crane_msg::msg::LuffingJibCraneMsg();
    lj_msg.crane_id = 3;
    lj_msg.crane_type = "luffingJibCrane";
    lj_msg.crane_x = luffing_crane_list[0].x;
    lj_msg.crane_y = luffing_crane_list[0].y;
    lj_msg.crane_z = luffing_crane_list[0].h;
    lj_msg.boom_length = luffing_crane_list[0].jib_length;
    lj_msg.boom_hor_angle = luffing_crane_list[0].slewing_angle / 180 * M_PI;
    lj_msg.boom_ver_angle = luffing_crane_list[0].jib_angle / 180 * M_PI;
    lj_msg.hook_height = luffing_crane_list[0].hoisting_height;
    msg.luffing_jib_crane_msgs.push_back(lj_msg);

    tc_msg.crane_id = 2;
    tc_msg.crane_type = "towerCrane";
    tc_msg.crane_x = tower_crane_list[0].x;
    tc_msg.crane_y = tower_crane_list[0].y;
    tc_msg.crane_z = tower_crane_list[0].h;
    tc_msg.boom_length = tower_crane_list[0].jib_length;
    tc_msg.boom_angle = tower_crane_list[0].slewing_angle / 180 * M_PI;
    tc_msg.trolley_radius = tower_crane_list[0].trolley_radius;
    tc_msg.hook_height = tower_crane_list[0].hoisting_height;
    msg.tower_crane_msgs.push_back(tc_msg);

    publisher->publish(msg);
    

    std::this_thread::sleep_for(100ms);
  }
  
  restoreTerminalSettings();

  rclcpp::shutdown();

  return 0;
}
