/**
 * @file test_udp2.cpp
 * @brief non-blocking UDP communication test and motor initialization
 * @author Haowen Liang(1224559437@qq.com)
 * @date 2025-5-28
 *
 * @copyright Copyright (C) 2025.
 *
 */
/* related header files */
#include "udp_comm.h"
#include "motor_init.h"
#include <iostream>
#include <string>
#include <unistd.h>

// 全局变量定义
UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};

int main(int argc, char *argv[])
{
    if (!udp_comm.init())
    {
        printf("UDP 初始化失败\n");
        return 1;
    }
    
    // 直接调用 InitMotors 函数进行电机初始化
    InitMotors(udp_comm, udp_send_data, udp_receive_data);

    return 0;
}