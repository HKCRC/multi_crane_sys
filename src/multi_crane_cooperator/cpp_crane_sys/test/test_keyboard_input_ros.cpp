
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
#include "multi_crane_msg/msg/collision_msg.hpp"

// CraneJointState crane_state[4];
std::vector<CraneJointState> crane_state(4); 
// 全局互斥锁
std::mutex crane_state_mutex;

void convertCraneMsg(CraneAntiCollision& crane_collision, multi_crane_msg::msg::MultiCraneMsg &msg)
{   
    // std::vector<CraneConfig> crane_list = crane_collision.crane_list_;

    int id = 0;
    for(auto & config : crane_collision.crane_list_)
    {
        
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
        id++;
    }

    long unsigned int crane_num = crane_collision.crane_num;
    long unsigned int main_crane_id = crane_collision.main_crane_id_;
    std::vector<std::pair<u_int, u_int>> predict_collision_crane_pairs = crane_collision.predict_collision_crane_pairs;

    // int collision_prediction_adj_mat[crane_num][crane_num]={0};
    // multi_crane_msg::msg::MultiCraneCollisionMsg multi_crane_collision_msg;
    std::vector<uint8_t> collision_prediction(crane_num, 0);  

    for(auto pair : predict_collision_crane_pairs)
    {
        collision_prediction[pair.first]=1;
        collision_prediction[pair.second]=1;
    }

    msg.collision_msg.main_crane_id = main_crane_id;
    msg.collision_msg.crane_num = crane_num;
    msg.collision_msg.collision_prediction = collision_prediction;

}

void convertCraneCollisionMsg(CraneAntiCollision& crane_collision, multi_crane_msg::msg::CollisionMsg &msg)
{   
    long unsigned int crane_num = crane_collision.crane_num;
    long unsigned int main_crane_id = crane_collision.main_crane_id_;
    std::vector<std::pair<u_int, u_int>> predict_collision_crane_pairs = crane_collision.predict_collision_crane_pairs;

    // int collision_prediction_adj_mat[crane_num][crane_num]={0};
    // multi_crane_msg::msg::MultiCraneCollisionMsg multi_crane_collision_msg;
    std::vector<uint8_t> collision_prediction(crane_num, 0); 

    for(auto pair : predict_collision_crane_pairs)
    {
        collision_prediction[pair.first]=1;
        collision_prediction[pair.second]=1;
    }

    msg.main_crane_id = main_crane_id;
    msg.crane_num = crane_num;
    msg.collision_prediction = collision_prediction;
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

void signalHandler(int signum) {
    restoreTerminalSettings();
    std::cout << "\nExiting gracefully...\n";
    exit(signum);
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
            crane_state[0].slewing_velocity += 4.65;
            break;
        case 'd':
            // crane_state[0].slewing_angle -= 5.0f;
            crane_state[0].slewing_velocity -= 4.65;
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
            // crane_state[1].slewing_angle += 5.0f;
            crane_state[1].slewing_velocity += 4.65f;
            break;
        case 'k':
            // crane_state[1].slewing_angle -= 5.0f;
            crane_state[1].slewing_velocity -= 4.65f;
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
            // crane_state[3].slewing_angle += 5.0f;
            crane_state[3].slewing_velocity += 5.0f;
            break;
        case '6':
            // crane_state[3].slewing_angle -= 5.0f;
            crane_state[3].slewing_velocity -= 5.0f;
            break;
        case '8': 
            crane_state[3].jib_trolley += 5.0f;
            break;
        case '2':
            crane_state[3].jib_trolley -= 5.0f;
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
  restoreTerminalSettings(); // 确保退出时恢复设置
}

void keyboardSimulatorUpdate(std::vector<CraneJointState>& crane_state,CraneAntiCollision& crane_collision, double dt, double current_time) {

        std::vector<std::pair<u_int, u_int>> predict_collision_crane_pairs = crane_collision.predict_collision_crane_pairs;
        std::vector<uint8_t> collision_prediction(crane_collision.crane_num, 0);  

        for(auto pair : predict_collision_crane_pairs)
        {
            collision_prediction[pair.first]=1;
            collision_prediction[pair.second]=1;
        }

        for (size_t i = 0; i < crane_state.size(); ++i) {
            if (static_cast<int>(collision_prediction[i]) == 1) //stop update if there is collision
            {   
                // do nothing
                // crane_state[i].slewing_angle = crane_state[i].slewing_angle
                // crane_state[i].slewing_angle = std::fmod(crane_state[i].slewing_angle, 360.0);
            }
            else 
            {   
                crane_state[i].slewing_angle += crane_state[i].slewing_velocity * dt;
                crane_state[i].slewing_angle += crane_state[i].slewing_velocity * dt;
                crane_state[i].slewing_angle = std::fmod(crane_state[i].slewing_angle, 360.0);
            }
        }
}

void keyboardSimulatorUpdate(std::vector<CraneJointState>& crane_state, double dt, double current_time) {
        for (size_t i = 0; i < crane_state.size(); ++i) {
            crane_state[i].current_time  = current_time;
            crane_state[i].slewing_angle += crane_state[i].slewing_velocity * dt;
            crane_state[i].slewing_angle = std::fmod(crane_state[i].slewing_angle, 360.0);
        }
}

int main(int argc, char ** argv)
{
    signal(SIGINT, signalHandler); // 捕获异常SIGINT

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
    rclcpp::Publisher<multi_crane_msg::msg::CollisionMsg>::SharedPtr collision_publisher = node->create_publisher<multi_crane_msg::msg::CollisionMsg>("multi_crane_collision", 10);
    long int cnt = 0;
    // 初始化上一次的时间
    rclcpp::Time last_time_ros = node->now();
    double last_time = last_time_ros.seconds();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // 获取当前时间
        rclcpp::Time current_time_ros = node->now();

        double current_time = current_time_ros.seconds();
        
        // 计算时间增量（秒）
        double dt = (current_time - last_time);
        
        // 更新上一次的时间
        last_time = current_time;
        
        std::lock_guard<std::mutex> lock(crane_state_mutex);

        // 对速度进行积分得到角度
        // keyboardSimulatorUpdate(crane_state,crane_collision, dt);
        keyboardSimulatorUpdate(crane_state, dt, current_time);

        std::cout<<"test: ";

        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
        std::cout<<"Main crane ID: "<< crane_collision.main_crane_id_ << "  ---------"<< std::endl;
        std::cout<<"Crane 0: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_trolley: "<< crane_state[0].jib_trolley << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
        std::cout<<"Crane 1: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_trolley: "<< crane_state[1].jib_trolley << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
        std::cout<<"Crane 3: slewing_angle: "<< crane_state[3].slewing_angle << ", jib_trolley: "<< crane_state[3].jib_trolley << ", hoisting_height: "<< crane_state[3].hoisting_height << std::endl;
        // update cranes' state and slewing velocity
        crane_collision.updateAllCraneState(crane_state);
        crane_collision.updateCraneSlewingVelocity();
        
        std::cout<<"distance between cranes: "<<std::endl;
        crane_collision.showDistanceAll();

        std::cout<<"static collision status: "<<std::endl;
        crane_collision.checkCollisionAll(5.0, true);
        
        // std::cout<<"predict collision status: "<<std::endl;
        // crane_collision.predictCollisionAll(5.0, true);
        // crane_collision.predictCollisionMainCraneNeighbor(5.0, true);

        std::cout<<"conservative collision status: "<<std::endl;
        crane_collision.checkBTMainCraneAllowedMotion(30.0, 5.0);
        
        //send to crane collision viualizer
        multi_crane_msg::msg::MultiCraneMsg msg;
        convertCraneMsg(crane_collision, msg);
        publisher->publish(msg);

        //send to crane collision status
        multi_crane_msg::msg::CollisionMsg collision_msg;
        convertCraneCollisionMsg(crane_collision, collision_msg);
        std::cout << "Collision prediction array: ";
        for(const auto& pred : collision_msg.collision_prediction) {
            std::cout << static_cast<int>(pred) << " ";
        }
        std::cout << std::endl;
        collision_publisher->publish(collision_msg);

        usleep(100*1000); // microseconds
    }
}