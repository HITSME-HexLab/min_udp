/**
 * @file timer.cpp
 * @brief timer base on ros and system clock
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-10-04
 * @ref https://gist.github.com/mcleary/b0bf4fa88830ff7c882d
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "timer.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

/**
 * @brief start the timer
 */
void Timer::start(void)
{
    running_flag_ = true;
    switch (source_type_)
    {
    case TimerSourceType::kNone:
        break;
    case TimerSourceType::kRos:
        ros_start_time_ = ros::Time::now();
        break;
    case TimerSourceType::kSystem:
        sys_start_time_ = std::chrono::high_resolution_clock::now();
        break;
    }
}

/**
 * @brief stop the timer
 */
void Timer::stop(void)
{
    running_flag_ = false;
    switch (source_type_)
    {
    case TimerSourceType::kNone:
        break;
    case TimerSourceType::kRos:
        ros_stop_time_ = ros::Time::now();
        break;
    case TimerSourceType::kSystem:
        sys_stop_time_ = std::chrono::high_resolution_clock::now();
        break;
    }
}

/**
 * @brief get the time between timer start and stop
 * @return the time, uint: millisecond
 */
double Timer::elapsedMilliseconds(void) const
{
    switch (source_type_)
    {
    case TimerSourceType::kNone:
        return 0;
    case TimerSourceType::kRos:
        if (running_flag_)
            return static_cast<double>(ros::Duration(ros::Time::now() - ros_start_time_).toNSec()) /
                   1000000.;
        else
            return static_cast<double>(ros::Duration(ros_stop_time_ - ros_start_time_).toNSec()) /
                   1000000.;
    case TimerSourceType::kSystem:
        if (running_flag_)
            return std::chrono::duration<double, std::milli>(
                       std::chrono::high_resolution_clock::now() - sys_start_time_)
                .count();
        else
            return std::chrono::duration<double, std::milli>(sys_stop_time_ - sys_start_time_)
                .count();
    }
    return 0;
}

/**
 * @brief get the time between timer start and stop
 * @return the time, uint: second
 */
double Timer::elapsedSeconds(void) const
{
    return elapsedMilliseconds() / 1000.;
}
