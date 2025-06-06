
#include <string>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>

#include "multi_crane_sys/collision_detection.hpp"
#include "multi_crane_sys/crane_utility.h"

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
            crane_state[3].slewing_angle += 5.0f;
            break;
        case '6':
            crane_state[3].slewing_angle -= 5.0f;
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
    long unsigned int cnt = 0;
    while(true)
    {
        std::cout<<"----------iterator: "<< cnt++ << "  ---------"<< std::endl;
        std::cout<<"Crane 1: slewing_angle: "<< crane_state[0].slewing_angle << ", jib_trolley: "<< crane_state[0].jib_trolley << ", hoisting_height: "<< crane_state[0].hoisting_height << std::endl;
        std::cout<<"Crane 2: slewing_angle: "<< crane_state[1].slewing_angle << ", jib_trolley: "<< crane_state[1].jib_trolley << ", hoisting_height: "<< crane_state[1].hoisting_height << std::endl;
        std::cout<<"Crane 4: slewing_angle: "<< crane_state[3].slewing_angle << ", jib_trolley: "<< crane_state[3].jib_trolley << ", hoisting_height: "<< crane_state[3].hoisting_height << std::endl;

        crane_collision.updateSingleCraneState(0, crane_state[0].slewing_angle, crane_state[0].jib_trolley, crane_state[0].hoisting_height);
        crane_collision.updateSingleCraneState(1, crane_state[1].slewing_angle, crane_state[1].jib_trolley, crane_state[1].hoisting_height);
        crane_collision.updateSingleCraneState(2, crane_state[3].slewing_angle, crane_state[3].jib_trolley, crane_state[3].hoisting_height);
        
        std::cout<<"distance between cranes: "<<std::endl;
        crane_collision.showDistanceAll();

        std::cout<<"collision status: "<<std::endl;
        crane_collision.checkCollisionAll(5.0, true);
        
        usleep(100*1000); // microseconds
    }
}