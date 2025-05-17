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


/* c++ standard library header files */
#include <iostream>
#include <string>

UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};


int main(int argc, char *argv[])
{
    // udp initialization
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }
    while (true)
    {
        //UDP通信
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kIdle);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        if (udp_comm.receive(1000))
        {
            udp_receive_data = udp_comm.getReceiveData();
            std::cout << "UDP receive successfully" << std::endl;
        }
        else
        {
            std::cout << "UDP receive timeout" << std::endl;
        }
    
    }
    return 0;
}
