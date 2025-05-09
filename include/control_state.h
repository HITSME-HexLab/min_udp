/**
 * @file control_state.h
 * @brief control state
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-10-28
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <Eigen/Dense>

/* internal project header files */
#include "robot_state.h"

namespace control_state
{
struct JointState
{
    JointState(const int leg_num, const int leg_dof)
    {
        kp = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        kd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        pos_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        vel_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        torque_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        pos_fdb = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        kp_default = Eigen::VectorXd::Zero(leg_dof);
        kd_default = Eigen::VectorXd::Zero(leg_dof);
    }
    Eigen::VectorXd kp_default, kd_default;
    Eigen::MatrixXd kp, kd;
    Eigen::MatrixXd pos_cmd, vel_cmd, torque_cmd;
    Eigen::MatrixXd pos_fdb;
};
struct FootState
{
    FootState(const int leg_num, const int leg_dof)
    {
        pos_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        vel_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        acc_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
        force_cmd = Eigen::MatrixXd::Zero(leg_dof, leg_num);
    }
    Eigen::MatrixXd pos_cmd, vel_cmd, acc_cmd, force_cmd;
};
} // namespace control_state

struct ControlState
{
    explicit ControlState(const RobotParam &robot_param)
        : foot(robot_param.leg_num, robot_param.leg_dof),
          joint(robot_param.leg_num, robot_param.leg_dof)
    {
        motor_init_torque.setZero();
    }
    virtual ~ControlState() = default;

    int control_frequency{}; // real time
    double control_dt{};     // ros simulated time

    control_state::FootState foot;
    control_state::JointState joint;

    // motor initialization related
    Eigen::Vector3d motor_init_torque;
    int motor_init_frequency{};
};
