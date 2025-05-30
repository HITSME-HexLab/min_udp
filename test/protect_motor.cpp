/**
 * @file protect_motor.cpp
 * @brief protect motor test
 * @author Haowen Liang(1224559437@qq.com)
 * @date 2025-5-20
 *
 * @copyright Copyright (C) 2025.
 *
 */
/* related header files */
#include "udp_comm.h"
#include "motor_init.h"
/* c system header files */

/* c++ standard library header files */
#include <iostream>
#include <string>

UdpComm udp_comm(8888, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};

int main(int argc, char *argv[])
{

    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }    

    while (true)
    {
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        ProtectMotors(udp_comm, udp_send_data, udp_receive_data);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        usleep(10000);
    }
    return 0;
}