/**
 * File: Timer.hpp
 * Author: Jing Yonglin
 * Description: Class definition for a simple timer
 */

#ifndef LIDAR_ALGORITHM_TIMER
#define LIDAR_ALGORITHM_TIMER

#include <chrono>

class Timer{
public:
    Timer() = default;

    void Start();
    int64_t Stop();
private:
    std::chrono::steady_clock::time_point start_pt_;
};

#endif