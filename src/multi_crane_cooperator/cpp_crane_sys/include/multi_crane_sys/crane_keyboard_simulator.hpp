#ifndef CRANE_KEYBOARD_SIMULATOR
#define CRANE_KEYBOARD_SIMULATOR

// src/multi_crane_cooperator/cpp_crane_sys/include/multi_crane_sys/crane_keyboard_driver.hpp
#pragma once

#include <string>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <signal.h>
#include "multi_crane_sys/crane_utility.h"

class CraneKeyboardSimulator {
public:
    CraneKeyboardSimulator() : should_exit_(false) {
        // 初始化起重机状态
        for (auto& state : crane_state_) {
            state.slewing_angle = 0.0;
            state.jib_angle = 0.0;
            state.hoisting_height = 0.0;
            state.slewing_velocity = 0.0;
        }
        
        // 设置信号处理
        signal(SIGINT, signalHandler);
    }

    ~CraneKeyboardSimulator() {
        restoreTerminalSettings();
    }

    // 开始键盘监听线程
    void start() {
        keyboard_thread_ = std::thread(&CraneKeyboardDriver::keyboardListener, this);
    }

    // 停止键盘监听线程
    void stop() {
        should_exit_ = true;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

    // 获取起重机状态
    void getCraneState(CraneJointState* state, int crane_id) {
        std::lock_guard<std::mutex> lock(crane_state_mutex_);
        *state = crane_state_[crane_id];
    }

    // 更新起重机状态
    void updateCraneState(int crane_id, double dt) {
        std::lock_guard<std::mutex> lock(crane_state_mutex_);
        crane_state_[crane_id].slewing_angle += crane_state_[crane_id].slewing_velocity * dt;
        crane_state_[crane_id].slewing_angle = std::fmod(crane_state_[crane_id].slewing_angle, 360.0);
    }

private:
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
                crane_state[1].slewing_velocity += 4.65f;
                break;
            case 'k':
                // crane_state[1].slewing_angle -= 5.0f;
                crane_state[1].slewing_velocity -= 4.65f;
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
    restoreTerminalSettings(); // 确保退出时恢复设置
    }

    std::thread keyboard_thread_;
    std::mutex crane_state_mutex_;
    CraneJointState crane_state_[4];
    termios original_tty_;
    static bool should_exit_;
};

// 静态成员初始化
// bool CraneKeyboardDriver::should_exit_ = false;

#endif