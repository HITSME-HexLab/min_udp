/**
 * @file timer.h
 * @brief timer base on ros and system clock
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-10-04
 * @ref https://gist.github.com/mcleary/b0bf4fa88830ff7c882d
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <chrono>
#include <ctime>

/* external project header files */
#include <ros/ros.h>

/* internal project header files */

enum class TimerSourceType
{
    kNone,
    kRos,
    kSystem
};

class Timer
{
  public:
    explicit Timer(TimerSourceType source_type) : source_type_(source_type){};
    ~Timer() = default;
    void start(void);
    void stop(void);
    [[nodiscard]] double elapsedMilliseconds(void) const;
    [[nodiscard]] double elapsedSeconds(void) const;

  private:
    TimerSourceType source_type_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> sys_start_time_, sys_stop_time_;
    ros::Time ros_start_time_, ros_stop_time_;
    bool running_flag_{};
};
