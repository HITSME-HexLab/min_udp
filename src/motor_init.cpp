/**
 * @file test_udp# 不再需要extern声明，由于我们通过引用传递对象p
 * @brief non-blocking UDP communication test and motor initialization
 * @author Haowen Liang(1224559437@qq.com)
 * @date 2025-5-20
 *
 * @copyright Copyright (C) 2025.
 *
 */
#include "motor_init.h"
#include "udp_comm.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm> 
#include <unistd.h>

// 不再需要extern UdpComm声明，由于我们通过引用传递UdpComm对象
extern udp::ReceiveData udp_receive_data;
extern udp::SendData udp_send_data;

// 初始化电机零点位置数组
double motorZeroPositions[18] = {0.0};

// 标记需要反转的电机，这些电机的命令方向需要取反
std::vector<int> reverseMotors = {1, 2, 4, 5, 9, 12, 15, 16, 17};

// 设置电机零点位置
void setMotorZeroPositions(const double* zeroPositions)
{
    // 不再使用全局udp_comm对象，由调用者确保在必要时接收数据
    if (zeroPositions != nullptr)
    {
        for(int i = 0; i < 18; ++i)
        {
            motorZeroPositions[i] = zeroPositions[i];
        }
    }
}

// 获取电机零点位置
double* getMotorZeroPositions()
{
    return motorZeroPositions;
}

// 判断电机是否需要反向
bool isMotorReversed(int motorIndex)
{
    return std::find(reverseMotors.begin(), reverseMotors.end(), motorIndex) != reverseMotors.end();
}

// 发送单个电机命令
void sendMotorCommand(int motorIndex, float jointAngle, float kp, float kd, float torque, udp::SendData &send_data)
{
    if (motorIndex < 0 || motorIndex >= 18)
    {
        std::cerr << "错误：电机索引 " << motorIndex << " 超出范围 [0-17]" << std::endl;
        return;
    }
    
    // 添加零点补偿
    float targetPos = motorZeroPositions[motorIndex];
    
    // 根据电机是否需要反向决定角度的加减方向
    if (isMotorReversed(motorIndex))
    {
        targetPos -= jointAngle; // 反向电机减去目标角度
    }
    else
    {
        targetPos += jointAngle; // 正向电机加上目标角度
    }
    
    // 设置电机命令
    send_data.udp_motor_send[motorIndex].pos = targetPos;
    send_data.udp_motor_send[motorIndex].kp = kp;
    send_data.udp_motor_send[motorIndex].kd = kd;
    send_data.udp_motor_send[motorIndex].torque = torque;
    
    // 设置状态为正常
    send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
    // 注意：不再直接发送，由调用者负责发送
}

// 发送单条腿的所有关节命令
void sendLegCommand(int legIndex, float hipAngle, float kneeAngle, float ankleAngle, 
                   float kp, float kd, float torque, udp::SendData &send_data)
{
    if (legIndex < 0 || legIndex >= 6)
    {
        std::cerr << "错误：腿索引 " << legIndex << " 超出范围 [0-5]" << std::endl;
        return;
    }
    
    // 计算该腿的三个关节对应的电机索引
    int hipMotorIdx = legIndex * 3;
    int kneeMotorIdx = legIndex * 3 + 1;
    int ankleMotorIdx = legIndex * 3 + 2;
    
    // 使用sendMotorCommand为每个关节设置命令
    // 髋关节命令
    sendMotorCommand(hipMotorIdx, hipAngle, kp, kd, torque, send_data);
    
    // 膝关节命令
    sendMotorCommand(kneeMotorIdx, kneeAngle, kp, kd, torque, send_data);
    
    // 踝关节命令
    sendMotorCommand(ankleMotorIdx, ankleAngle, kp, kd, torque, send_data);
}

void InitMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    Timer timer_;
    int calibrationStep = -1;
    int forceControlTime = 0;
    int joint_num = 0;
    int sum_flag = 0;
    
    // 将const变量和静态变量移到switch外面，避免跨变量初始化跳转
    const int maxK = 200;  // 定义根关节最大插值次数
    static int kk = 0;     // 根关节静态计数器
    const int maxK2 = 200; // 定义膝关节和踝关节最大插值次数
    static int kk2 = 0;    // 膝关节和踝关节静态计数器

    Eigen::Matrix<bool, 3, 6> init_finished_flag;
    Eigen::MatrixXd current_pos(3, 6);
    Eigen::MatrixXd last_pos(3, 6);
    // 使用全局已定义的reverseMotors变量，不要重新定义

    while (true)
    {
        timer_.start();

        switch (calibrationStep)
        {
        case -1: // 给定力控命令
            forceControlTime++;
            
            udp_comm.receive(10);
            for (int j = 0; j < 6; ++j)
            {
                int motorIndex = joint_num + 3 * j; // 计算当前电机索引
                send_data.udp_motor_send[motorIndex].kp = 0.0;
                if (joint_num == 0)
                {
                    bool isReverseMotor = std::find(reverseMotors.begin(), reverseMotors.end(), motorIndex) != reverseMotors.end();
                    double appliedTorque = isReverseMotor ? -0.2 : 0.2;
                    send_data.udp_motor_send[motorIndex].torque = appliedTorque;
                    send_data.udp_motor_send[motorIndex].kd = 0.5;
                }
                std::cout << "forceControlTime: " << forceControlTime << std::endl;
            }
            if (forceControlTime > 200)
            {
                calibrationStep = 0;
                forceControlTime = 0;
            }
            break;

        case 0: // 判断是否到达机械限位
            init_finished_flag.setZero();
            // 收取关节数据
            if (udp_comm.receive(10))
            {
                receive_data = udp_comm.getReceiveData();
                // 遍历所有电机并给矩阵赋值
                for (int leg = 0; leg < 6; leg++)
                { // 遍历6条腿
                    for (int joint = 0; joint < 1; joint++)
                    { // 遍历每条腿的3个关节
                        // 计算电机索引 (每条腿3个关节)
                        int motor_idx = leg * 3 + joint;
                        // 将电机位置值赋给对应矩阵元素
                        current_pos(joint, leg) = udp_receive_data.udp_motor_receive[motor_idx].pos;
                    }
                }
            }

            for (int j = 0; j < 6; ++j)
            {
                if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.002)
                {
                    init_finished_flag(joint_num, j) = true;
                }
            }
            //  determine whether all six joints in the same position reached the mechanical limit
            sum_flag = 0;
            for (int j = 0; j < 6; ++j)
            {
                sum_flag += init_finished_flag(joint_num, j);
            }
            if (sum_flag == 6)
            {
                std::cout << "test ok " << std::endl;
                // 保存电机零位
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = joint_num + 3 * j; // 在循环内部声明motorIndex
                    // 记录零位位置
                    double zeroPosition = current_pos(joint_num, j);
                    motorZeroPositions[motorIndex] = zeroPosition; // 保存到全局零点数组
                    udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                    std::cout << "Joint " << joint_num << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                }
                calibrationStep = 1; // 进入根关节回0度case
            }
            break;

        case 1: // 根关节回0度位置        
            //std::cout << "进入根关节回0度状态" << std::endl;
            
            kk++;
            if (kk > maxK)
                kk = maxK;
            
            if (udp_comm.receive(10))
            {
                udp_receive_data = udp_comm.getReceiveData();

                for (int j = 0; j < 6; ++j) // 6条腿
                {
                    int motorIndex = 3 * j; // 计算电机索引
                    float targetDiff;

                    // 根据电机编号设置目标位置差值
                    if (motorIndex == 0 || motorIndex == 3 || motorIndex == 6)
                    {
                        targetDiff = -7.25f;
                    }
                    else if (motorIndex == 9 || motorIndex == 12 || motorIndex == 15)
                    {
                        targetDiff = 7.25f;
                    }

                    // 进行线性插值
                    float tmpTgt = current_pos(joint_num, j) + (targetDiff / float(maxK)) * kk;

                    // 设置电机命令
                    udp_send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                    udp_send_data.udp_motor_send[motorIndex].kp = 0.05; // 位置环增益
                    udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                }
            }
            if (kk == maxK) // 完成回零过程
            {
                calibrationStep = 2;
            }        
        break;

        case 2:
            //std::cout << "进入膝关节和踝关节初始化状态" << std::endl;
            forceControlTime++;
            
            udp_comm.receive(10);
            
            for (int j = 0; j < 6; ++j)
            {
                // 处理膝关节(joint_num = 1)
                int kneeMotorIndex = 1 + 3 * j;  // 计算膝关节电机索引
                udp_send_data.udp_motor_send[kneeMotorIndex].kp = 0.0;
                bool isKneeReverse = std::find(reverseMotors.begin(), reverseMotors.end(), kneeMotorIndex) != reverseMotors.end();
                double kneeAppliedTorque = isKneeReverse ? -0.5 : 0.5;
                udp_send_data.udp_motor_send[kneeMotorIndex].torque = kneeAppliedTorque;
                udp_send_data.udp_motor_send[kneeMotorIndex].kd = 1.5;
                
                // 处理踝关节(joint_num = 2)
                int ankleMotorIndex = 2 + 3 * j;  // 计算踝关节电机索引
                udp_send_data.udp_motor_send[ankleMotorIndex].kp = 0.0;
                bool isAnkleReverse = std::find(reverseMotors.begin(), reverseMotors.end(), ankleMotorIndex) != reverseMotors.end();
                double ankleAppliedTorque = isAnkleReverse ? -0.25 : 0.25;
                udp_send_data.udp_motor_send[ankleMotorIndex].torque = ankleAppliedTorque;
                udp_send_data.udp_motor_send[ankleMotorIndex].kd = 0.5;
            }
            
            if (forceControlTime > 200)
            {
                calibrationStep = 3;  // 完成所有关节初始化，进入保护状态
                forceControlTime = 0;
            }        
        break;
        
        case 3:
            //std::cout << "进入膝关节和踝关节机械限位状态" << std::endl;            
            init_finished_flag.setZero();
            
            if (udp_comm.receive(10))
            {
                udp_receive_data = udp_comm.getReceiveData();
                for (int leg = 0; leg < 6; leg++)
                { 
                    for (int joint = 1; joint < 3; joint++)
                    {                         
                        int motor_idx = leg * 3 + joint;                        
                        current_pos(joint, leg) = udp_receive_data.udp_motor_receive[motor_idx].pos;
                    }
                }
            }

            // 合并检查膝关节和踝关节是否到达限位的循环
            for (int joint = 1; joint < 3; joint++)
            {
                for (int j = 0; j < 6; ++j)
                {
                    if (std::abs(current_pos(joint, j) - last_pos(joint, j)) < 0.002)
                    {
                        init_finished_flag(joint, j) = true;
                    }
                }
            }
            
            sum_flag = 0;
            // 计算膝关节和踝关节限位标志总和
            for (int joint = 1; joint < 3; joint++)
            {
                for (int j = 0; j < 6; ++j)
                {
                    sum_flag += init_finished_flag(joint, j);
                }
            }
            
            if (sum_flag == 12) // 6条腿*2个关节=12个关节都到达限位
            {
                std::cout << "test ok " << std::endl;
                // 保存膝关节和踝关节零位
                for (int joint = 1; joint < 3; joint++)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        int motorIndex = joint + 3 * j; 
                        double zeroPosition = current_pos(joint, j);
                        motorZeroPositions[motorIndex] = zeroPosition; // 保存到全局零点数组
                        udp_send_data.udp_motor_send[motorIndex].torque = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.0;
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.0;
                        std::cout << "Joint " << joint << " Leg " << j << " zero position set to: " << zeroPosition << std::endl;
                    }
                }
                
                calibrationStep = 4; 
            }
            break;

        case 4: // 膝关节和踝关节回0度位置            
            //std::cout << "进入膝关节和踝关节回0度位置状态" << std::endl;
            kk2++;
            if(kk2 > maxK2) kk2 = maxK2;            
            
            for(int curr_joint = 1; curr_joint <= 2; curr_joint++) 
            {
                for (int j = 0; j < 6; ++j)
                {
                    int motorIndex = curr_joint + 3*j;
                    float targetDiff;                    
                    bool isReverseMotor = std::find(reverseMotors.begin(), 
                                                reverseMotors.end(), 
                                                motorIndex) != reverseMotors.end();
                    if (curr_joint == 1)                    
                        targetDiff = isReverseMotor ? 5.25f : -5.25f;                    
                    else                     
                        targetDiff = isReverseMotor ? 27.0f : -27.0f;                    

                    float tmpTgt = current_pos(curr_joint, j) + (targetDiff / float(maxK2)) * kk2;

                    udp_send_data.udp_motor_send[motorIndex].pos = tmpTgt;
                    if (curr_joint == 1) //knee joint
                    {
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.05;   
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.5;
                    } 
                    else                 //ankle joint
                    { 
                        udp_send_data.udp_motor_send[motorIndex].kp = 0.05;  
                        udp_send_data.udp_motor_send[motorIndex].kd = 0.5;  
                    }
                }
            }
            
        break;
        }
        
        last_pos = current_pos;

        send_data.state = static_cast<uint8_t>(CommBoardState::kNormal);
        udp_comm.setSendData(send_data);
        udp_comm.send();

        timer_.stop();
        float control_frequency = 100.0;
        double time_compensate =
            1000. / control_frequency - timer_.elapsedMilliseconds();

        if (time_compensate > 0)
        {
            usleep(static_cast<int>(time_compensate * 1000));
        }
    }
}

void ProtectMotors(UdpComm &udp_comm, udp::SendData &send_data, udp::ReceiveData &receive_data)
{
    for(int i = 0; i < 18; ++i)
    {
        send_data.udp_motor_send[i].torque = 0.0;
        send_data.udp_motor_send[i].kp = 0.0;
        send_data.udp_motor_send[i].kd = 0.0;
    }    
    udp_comm.setSendData(send_data);
    udp_comm.send();
}