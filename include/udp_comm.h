/**
 * @file udp_comm.h
 * @brief UDP communication interface
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-19
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */

/* c system header files */
#include <memory>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

/* c++ standard library header files */
#include <cerrno>
#include <cstring>

/* external project header files */

/* internal project header files */
#include "udp_basic.h"

namespace udp
{
/**
 * @brief motor command (rotor)
 */
typedef struct
{
    // uint8_t mode;
    float torque; // unit: Nm
    float vel;    // unit: rad/s
    float pos;    // unit: rad
    float kp;
    float kd;
} __attribute__((packed)) MotorSend;

/**
 * @brief motor feedback (rotor)
 */
typedef struct
{
    float pos; // unit: rad
    float vel; // unit: rad/s
    float acc;
    float torque; // unit: Nm
    float temp;
} __attribute__((packed)) MotorReceive;

/**
 * @brief receive from HexapodCommBoard through UDP
 */
typedef struct
{
    uint8_t header[2];
    uint8_t state;
    MotorReceive udp_motor_receive[18];
    // uint8_t foot_contact[6]; // Foot contact
    uint16_t foot_pressure[6];
    uint32_t check_digit;
} __attribute__((packed)) ReceiveData;

/**
 * @brief send to HexapodCommBoard through UDP
 */
typedef struct
{
    uint8_t header[2];
    uint8_t state;
    MotorSend udp_motor_send[18];
    uint32_t check_digit;
} __attribute__((packed)) SendData;
} // namespace udp

class UdpComm
{
  public:
    UdpComm(uint16_t local_port, const std::string &target_ip, uint16_t target_port);
    ~UdpComm() = default;
    bool init(void);
    bool receive(int32_t max_wait_ms = -1); // default: blocking(-1)
    bool send(void);
    void setSendData(const udp::SendData &data);
    [[nodiscard]] const udp::ReceiveData &getReceiveData(void) const;

  private:
    uint16_t local_port_{}, target_port_{};
    std::string target_ip_{};
    std::unique_ptr<udp_client_server::udp_server> p_server_ = nullptr;
    std::unique_ptr<udp_client_server::udp_client> p_client_ = nullptr;
    uint8_t receive_buffer_[sizeof(udp::ReceiveData)]{};
    udp::SendData udp_send_data_{};
    udp::ReceiveData udp_receive_data_{};
};
