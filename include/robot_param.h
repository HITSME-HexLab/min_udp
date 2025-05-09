/**
 * @file robot_param.h
 * @brief robot parameters
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-11-29
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

enum class MotorInitType
{
    /* [Robot hanging up] The motor rotates with 'motor_init_torque' until it hits the mechanical limit,
       and set this position to 'joint_calibration_pos' */
    kAutoCalibration,
    /* [Robot lying on the ground with thighs right in middle]
       The motor rotates with 'motor_init_torque' until it hits the mechanical limit(knee) or the ground(foot),
       and set this position to 'joint_calibration_pos' */
    kLyingAutoCalibration,
    /* Manually move the motor to a fixed position, and set this position
       to 'joint_calibration_pos' */
    kManualCalibration,
    /* The motor has absolute encoder, so use its raw feedback */
    kAbsoluteEncoder,
    /* [Robot lying on calibration shelf][ElSpiderAir ONLY]
        FIXME: Ghost values exists
       The motor rotates with 'motor_init_torque' until it hits the mechanical limit,
       and set this position to 'joint_calibration_pos' */
    kOnCalibrationShelf
};

struct RobotParam
{
    RobotParam(void);
    ~RobotParam() = default;

    // robot
    std::string robot_name{};
    std::string robot_type{};
    Eigen::MatrixXd hip_offset;
    Eigen::Vector3d link_length;
    Eigen::Matrix3d trunk_inertia;
    Eigen::Vector3d max_step_size;
    bool with_foot_force_sensor{false};
    double mass{};

    // leg
    int leg_num{}, leg_dof{};
    Eigen::MatrixXd leg_mirror_coe;

    // joint
    Eigen::Vector3d joint_protect_pos_max, joint_protect_pos_min;
    Eigen::MatrixXd joint_calibration_pos;

    // motor
    Eigen::Vector3d max_motor_torque;
    MotorInitType motor_init_type;
    Eigen::Vector3d external_reduction_ratio, motor_reduction_ratio;
};

inline RobotParam::RobotParam(void)
{
    // robot
    hip_offset.setZero();
    trunk_inertia.setZero();
    link_length.setZero();
    max_step_size.setZero();

    // leg
    leg_mirror_coe.setZero();

    // joint
    joint_calibration_pos.setZero();
    joint_protect_pos_max.setZero();
    joint_protect_pos_min.setZero();

    // motor
    max_motor_torque.setZero();
    motor_init_type = MotorInitType::kAutoCalibration;
    external_reduction_ratio.setZero();
    motor_reduction_ratio.setZero();
}
