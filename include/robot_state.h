/**
 * @file robot_state.h
 * @brief robot state
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-28
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <array>
#include <atomic>
#include <shared_mutex>

/* external project header files */
#include <Eigen/Dense>

/* internal project header files */
#include "robot_param.h"

enum class CommBoardState : uint8_t
{
    kIdle,   /* disable motors */
    kNormal, /* enable motors */
    kError,  /* enter damping mode to protect robot */
};

enum class VibrationType
{
    kType1,
    kType2,
    kType3
};

namespace robot_state
{
struct JointState
{
    JointState(const int leg_num, const int leg_dof)
    {
        using namespace Eigen;
        pos_zero = MatrixXd::Zero(leg_dof, leg_num);
        pos_cmd = MatrixXd::Zero(leg_dof, leg_num);
        vel_cmd = MatrixXd::Zero(leg_dof, leg_num);
        torque_cmd = MatrixXd::Zero(leg_dof, leg_num);
        kp = MatrixXd::Zero(leg_dof, leg_num);
        kd = MatrixXd::Zero(leg_dof, leg_num);
        pos_raw_fdb = MatrixXd::Zero(leg_dof, leg_num);
        pos_fdb = MatrixXd::Zero(leg_dof, leg_num);
        vel_fdb = MatrixXd::Zero(leg_dof, leg_num);
        acc_fdb = MatrixXd::Zero(leg_dof, leg_num);
        torque_fdb = MatrixXd::Zero(leg_dof, leg_num);
        temp_fdb = MatrixXd::Zero(leg_dof, leg_num);
    }
    /**
     * @brief joint(motor shaft) command, unit: rad, rad/s, Nm
     */
    Eigen::MatrixXd pos_zero;
    Eigen::MatrixXd pos_cmd, vel_cmd, torque_cmd;
    Eigen::MatrixXd kp, kd;

    /**
     * @brief joint(motor shaft) feedback, unit: rad, rad/s, Nm
     */
    Eigen::MatrixXd pos_raw_fdb, pos_fdb;
    Eigen::MatrixXd vel_fdb, acc_fdb, torque_fdb, temp_fdb;
};
struct FootState
{
    FootState(const int leg_num, const int leg_dof)
    {
        using namespace Eigen;
        pos_cmd = MatrixXd::Zero(leg_dof, leg_num);

        pos_cmd = MatrixXd::Zero(leg_dof, leg_num);
        vel_cmd = MatrixXd::Zero(leg_dof, leg_num);
        acc_cmd = MatrixXd::Zero(leg_dof, leg_num);
        force_cmd = MatrixXd::Zero(leg_dof, leg_num);
        pos_fdb = MatrixXd::Zero(leg_dof, leg_num);
        vel_fdb = MatrixXd::Zero(leg_dof, leg_num);
        acc_fdb = MatrixXd::Zero(leg_dof, leg_num);
        force_fdb = MatrixXd::Zero(leg_dof, leg_num);
        pos_world_fdb = MatrixXd::Zero(leg_dof, leg_num);
        vel_world_fdb = MatrixXd::Zero(leg_dof, leg_num);
        acc_world_fdb = MatrixXd::Zero(leg_dof, leg_num);
        force_world_fdb = MatrixXd::Zero(leg_dof, leg_num);
        pos_base_fdb = MatrixXd::Zero(leg_dof, leg_num);

        plan_contact_state = Matrix<bool, Dynamic, Dynamic>::Zero(leg_num, 1);
        estimated_contact_state = Matrix<bool, Dynamic, Dynamic>::Zero(leg_num, 1);
    }
    /**
     * @brief foot command in [robot frame], only for debug. unit: m, m/s, N
     */
    Eigen::MatrixXd pos_cmd, vel_cmd, acc_cmd, force_cmd;

    /**
     * @brief foot feedback in [robot frame], unit: m
     */
    Eigen::MatrixXd pos_fdb, vel_fdb, acc_fdb, force_fdb;

    /**
     * @brief foot feedback in [world frame], unit: m
     */
    Eigen::MatrixXd pos_world_fdb, vel_world_fdb, acc_world_fdb, force_world_fdb;

    /**
     * @brief foot feedback in [base frame], unit: m
     *        base frame is centered at robot frame's origin but parallels to the world frame
     */
    Eigen::MatrixXd pos_base_fdb;

    /**
     * @brief plan and estimate contact state
     */
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> plan_contact_state,
        estimated_contact_state; // temp dimensionality
};
struct BodyState
{
    BodyState(void)
    {
        linear_pos_cmd.setZero();
        linear_vel_cmd.setZero();
        angular_pos_cmd.setZero();
        angular_vel_cmd.setZero();
        linear_pos_fdb.setZero();
        linear_vel_fdb.setZero();
        angular_pos_fdb.setZero();
        angular_vel_fdb.setZero();
        euler_angle.setZero();
        rot_mat.setZero();
        rot_mat_z.setZero();
        quat.setIdentity();
    }
    Eigen::Vector3d linear_pos_cmd, linear_vel_cmd, angular_pos_cmd, angular_vel_cmd;
    Eigen::Vector3d linear_pos_fdb, linear_vel_fdb, angular_pos_fdb, angular_vel_fdb;
    /* feedback */
    Eigen::Vector3d euler_angle; // equal to angular_pos_fdb
    Eigen::Matrix3d rot_mat, rot_mat_z;
    Eigen::Quaterniond quat;
};
struct ImuSensor
{
    ImuSensor(void)
    {
        angular_vel.setZero();
        linear_acc.setZero();
        quat.setIdentity();
        euler_angle.setZero();
    }
    Eigen::Vector3d angular_vel, linear_acc;
    Eigen::Quaterniond quat;
    Eigen::Vector3d euler_angle;
};
struct Joystick
{
    // use array instead of vector to guarantee atomic operations
    // use this order to avoid struct padding
    std::array<std::atomic<double>, 4> axes{};
    std::array<std::atomic<bool>, 12> buttons{};
    std::atomic<VibrationType> vibration_type{};
    std::atomic<bool> vibration_request{};
};
struct Odometry
{
    Odometry(void)
    {
        euler_angle.setZero();
        position.setZero();
        linear_vel.setZero();
        angular_vel.setZero();
        quat.setIdentity();
    }
    Eigen::Vector3d position, euler_angle, linear_vel, angular_vel;
    Eigen::Quaterniond quat;
};
}; // namespace robot_state

/**
 * @brief contains all robot state
 */
struct RobotState
{
    explicit RobotState(const RobotParam &robot_param);
    ~RobotState() = default;

    // robot
    RobotParam param;

    // system
    std::atomic<CommBoardState> comm_board_state{};
    std::atomic<bool> feedback_ready_flag{};
    std::atomic<bool> motor_init_finished_flag{}, motor_init_request{};
    std::atomic<bool> use_state_estimator{true};
    std::atomic<bool> use_vrpn{false};

    robot_state::FootState foot;
    robot_state::JointState joint;
    robot_state::BodyState body{};
    robot_state::ImuSensor imu{};
    robot_state::Joystick joystick{};
    robot_state::Odometry odometry{};

    /**
     * @brief multi thread shared_mutex
     */
    std::shared_mutex cmd_mutex, fdb_mutex;
};

/**
 * @brief Construct a new robot state::robot state object
 */
inline RobotState::RobotState(const RobotParam &robot_param)
    : param(robot_param), foot(robot_param.leg_num, robot_param.leg_dof),
      joint(robot_param.leg_num, robot_param.leg_dof)
{
    comm_board_state = CommBoardState::kIdle;
}
