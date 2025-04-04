
#include <string>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <mutex> // 添加互斥锁保护共享数据

#include "multi_crane_sys/collision_detection.hpp"
#include "multi_crane_sys/crane_utility.h"

#include "rclcpp/rclcpp.hpp"
#include "multi_crane_msg/msg/multi_crane_msg.hpp"
#include "multi_crane_msg/msg/luffing_jib_crane_msg.hpp"
#include "multi_crane_msg/msg/tower_crane_msg.hpp"

// 全局互斥锁
std::mutex crane_state_mutex;

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

CraneJointState crane_state[4];

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

    std::lock_guard<std::mutex> lock(crane_state_mutex);

    switch (input) {
        case 'a':
            // crane_state[0].slewing_angle += 5.0f;
            crane_state[0].slewing_velocity += 5.0f;
            break;
        case 'd':
            // crane_state[0].slewing_angle -= 5.0f;
            crane_state[0].slewing_velocity -= 5.0f;
            break;
        case 'w': 
            crane_state[0].jib_angle += 5.0f;
            break;
        case 's':
            crane_state[0].jib_angle -= 5.0f;
            break;
        case 'r': 
            crane_state[0].hoisting_height -= 1.0f;
            break;
        case 'f':
            crane_state[0].hoisting_height += 1.0f;
            break;
        case 'h':
            // crane_state[1].slewing_angle += 5.0f;
            crane_state[1].slewing_velocity += 5.0f;
            break;
        case 'k':
            // crane_state[1].slewing_angle -= 5.0f;
            crane_state[1].slewing_velocity -= 5.0f;
            break;
        case 'u': 
            crane_state[1].jib_angle += 5.0f;
            break;
        case 'j':
            crane_state[1].jib_angle -= 5.0f;
            break;
        case 'o': 
            crane_state[1].hoisting_height -= 1.0f;
            break;
        case 'l':
            crane_state[1].hoisting_height += 1.0f;
            break;
        case '4':
            // crane_state[3].slewing_angle += 5.0f;
            crane_state[3].slewing_velocity += 5.0f;
            break;
        case '6':
            // crane_state[3].slewing_angle -= 5.0f;
            crane_state[3].slewing_velocity -= 5.0f;
            break;
        case '8': 
            crane_state[3].jib_angle += 5.0f;
            break;
        case '2':
            crane_state[3].jib_angle -= 5.0f;
            break;
        case '-': 
            crane_state[3].hoisting_height -= 1.0f;
            break;
        case '+':
            crane_state[3].hoisting_height += 1.0f;
            break;
        default:
            std::cout << "Invalid input" << std::endl;
            break;
    }
    usleep(10*1000);
  }
}

int main(int argc, char ** argv)
{
    if (argc < 2) 
    {
            std::cerr << "Usage: " << argv[0] << " <path_to_param_file>\n";
            return 1;
    }

    std::string paramFilePath = (std::string) argv[1];

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

    std::unique_ptr<std::thread> thread_input = std::make_unique<std::thread>(&keyboardListener); // using keyboard to control crane status
    /******** ROS initialization *********/
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test");

    rclcpp::Publisher<multi_crane_msg::msg::MultiCraneMsg>::SharedPtr publisher = node->create_publisher<multi_crane_msg::msg::MultiCraneMsg>("multi_crane", 10);

    long int cnt = 0;
    // 初始化上一次的时间
    rclcpp::Time last_time = node->now();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // 获取当前时间
        rclcpp::Time current_time = node->now();
        
        // 计算时间增量（秒）
        double dt = (current_time - last_time).seconds();
        
        // 更新上一次的时间
        last_time = current_time;
        
        std::lock_guard<std::mutex> lock(crane_state_mutex);

        // 对速度进行积分得到角度
        crane_state[0].slewing_angle += crane_state[0].slewing_velocity * dt;
        crane_state[1].slewing_angle += crane_state[1].slewing_velocity * dt;
        crane_state[3].slewing_angle += crane_state[3].slewing_velocity * dt;

        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
        std::cout<<"Crane 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_angle: "<< crane_state[0].jib_angle << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
        std::cout<<"Crane 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_angle: "<< crane_state[1].jib_angle << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
        std::cout<<"Crane 4: slewing_angle: "<< crane_state[3].slewing_angle << ", jib_angle: "<< crane_state[3].jib_angle << ", hoisting_height: "<< crane_state[3].hoisting_height << std::endl;
        // update cranes' state
        crane_collision.updateCraneState(0, crane_state[0].slewing_angle, crane_state[0].jib_angle,\
         crane_state[0].hoisting_height, crane_state[0].slewing_velocity);
        crane_collision.updateCraneState(1, crane_state[1].slewing_angle, crane_state[1].jib_angle,\
         crane_state[1].hoisting_height, crane_state[1].slewing_velocity);
        crane_collision.updateCraneState(3, crane_state[3].slewing_angle, crane_state[3].jib_angle,\
         crane_state[3].hoisting_height, crane_state[3].slewing_velocity);
        
        std::cout<<"distance between cranes: "<<std::endl;
        crane_collision.showDistanceAll();

        std::cout<<"collision status: "<<std::endl;
        crane_collision.checkCollisionAll(5.0, true);
        
        std::cout<<"predict collision status: "<<std::endl;
        crane_collision.predictCollisionAll(5.0, true);

        multi_crane_msg::msg::MultiCraneMsg msg;
        convertCraneMsg(crane_collision.crane_list_, msg);
        publisher->publish(msg);

        usleep(100*1000); // microseconds
    }
}