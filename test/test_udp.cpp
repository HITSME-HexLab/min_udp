/**
 * @file test_udp_non_blocking.cpp
 * @brief non-blocking UDP communication test
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-19
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "udp_comm.h"

/* c system header files */

/* c++ standard library header files */
#include <iostream>
#include <string>

/* external project header files */
//#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>

/* internal project header files */
//#include "robot_state.h"

UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};
//sensor_msgs::JointState joint_state;

int main(int argc, char *argv[])
{
    //ros::init(argc, argv, "test_motor_udp");
    //ros::Rate loop_rate(100);
    // initialization
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }

    while (true)
    {
        // printf("%d\n", static_cast<uint8_t>(CommBoardState::ERROR));
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kIdle);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        if (udp_comm.receive(1000))
        {
            udp_receive_data = udp_comm.getReceiveData();
            std::cout << "receive successfully" << std::endl;
        }
        std::cout << "is running" << std::endl;
    }
    return 0;
}