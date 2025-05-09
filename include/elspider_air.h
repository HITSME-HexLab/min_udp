/**
 * @file elspider_air.h
 * @author Master Yip (2205929492@qq.com), Haoyu Wang (qrpucp@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */
#include "robot_param.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "basic_math.hpp"

/*
               head
  (leg3)LF ------------- RF(leg0)
           |           |
           |     x     |
  (leg4)LM |  y__|     | RM(leg1)
           |           |
           |           |
  (leg5)LB ------------- RB(leg2)
*/

inline RobotParam buildElspiderAir(void)
{
    using namespace basic_math;

    RobotParam param;

    /* robot */
    param.robot_name = "elspider_air";
    param.robot_type = "hexapod";
    param.leg_dof = 3;
    param.leg_num = 6;

    //===================================================================================
    // NOTE: these params are NOT USED since LeggedKinModule is deployed for kinematics
    /* leg */
    param.leg_mirror_coe.resize(param.leg_num, 1);
    param.leg_mirror_coe << 1, 1, 1, -1, -1, -1;
    param.link_length << 0.15, 0.13, 0.25;
    // rf--rm--rb--lf--lm--lb (offset relative to the center of the robot)
    param.hip_offset.resize(param.leg_dof, param.leg_num);
    // param.hip_offset.col(0) = Eigen::Vector3d(0.3, -0.08, 0.011);
    // param.hip_offset.col(1) = Eigen::Vector3d(0, -0.14, 0.011);
    // param.hip_offset.col(2) = Eigen::Vector3d(-0.3, -0.08, 0.011);
    // param.hip_offset.col(3) = Eigen::Vector3d(0.3, 0.08, 0.011);
    // param.hip_offset.col(4) = Eigen::Vector3d(0, 0.14, 0.011);
    // param.hip_offset.col(5) = Eigen::Vector3d(-0.3, 0.08, 0.011);
    // NOTE: considering asymmetric leg, 0.054m offset is applied for temp calibration
    param.hip_offset.col(0) = Eigen::Vector3d(0.354, -0.08, 0.011);
    param.hip_offset.col(1) = Eigen::Vector3d(0.054, -0.14, 0.011);
    param.hip_offset.col(2) = Eigen::Vector3d(-0.354, -0.08, 0.011);
    param.hip_offset.col(3) = Eigen::Vector3d(0.354, 0.08, 0.011);
    param.hip_offset.col(4) = Eigen::Vector3d(0.054, 0.14, 0.011);
    param.hip_offset.col(5) = Eigen::Vector3d(-0.354, 0.08, 0.011);
    // thigh z-axis offset
    for (int i = 0; i < param.leg_num; ++i)
    {
        param.hip_offset(2, i) -= 0.05;
    }
    //===================================================================================

    param.mass = 15.8991;

    double truck_ixx = 0.0926086;
    double truck_ixy = -0.00001892;
    double truck_ixz = 0.00000158;
    double truck_iyy = 0.47054913;
    double truck_iyz = 0.00000000;
    double truck_izz = 0.53896766;

    param.trunk_inertia << truck_ixx, truck_ixy, truck_ixz, truck_ixy, truck_iyy, truck_iyz,
        truck_ixz, truck_iyz, truck_izz;

    param.max_step_size << 0.175, 0.05, 0.1;

    param.with_foot_force_sensor = false;

    /* motor */
    param.external_reduction_ratio << 1, 1, 1;
    param.motor_reduction_ratio << 9.1, 9.1, 9.1;
    // Unitree A1 motor, 33.5 * 0.8 = 26.8
    param.max_motor_torque << 26.8, 26.8, 26.8;
    param.motor_init_type = MotorInitType::kLyingAutoCalibration;

    /* joint */
    param.joint_calibration_pos.resize(param.leg_dof, param.leg_num);
    Eigen::Matrix<double, 6, 3> joint_calibration_pos_T;
    // IMPORTANT: the defined zero point of the leg: when the leg is straight
    if (param.motor_init_type == MotorInitType::kAutoCalibration)
    {
        // clang-format off
        // Hardware
        joint_calibration_pos_T <<  deg2rad(-48), deg2rad(110), deg2rad(-229.93), // 0
                                    deg2rad(-48), deg2rad(110), deg2rad(-229.93), // 1
                                    deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 2
                                    deg2rad(-48), deg2rad(110), deg2rad(-229.93), // 3
                                    deg2rad(-48), deg2rad(110), deg2rad(-229.93), // 4
                                    deg2rad(-48), deg2rad(110), deg2rad(-209.93); // 5
        // Gazebo
        // joint_calibration_pos_T <<  deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 0
        //                             deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 1
        //                             deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 2
        //                             deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 3
        //                             deg2rad(-48), deg2rad(110), deg2rad(-209.93), // 4
        //                             deg2rad(-48), deg2rad(110), deg2rad(-209.93); // 5
        // clang-format on
    }
    else if (param.motor_init_type == MotorInitType::kLyingAutoCalibration)
    {
        // clang-format off
        joint_calibration_pos_T <<  deg2rad(0), deg2rad(110), deg2rad(-150), // 0
                                    deg2rad(0), deg2rad(110), deg2rad(-150), // 1
                                    deg2rad(0), deg2rad(110), deg2rad(-150), // 2
                                    deg2rad(0), deg2rad(110), deg2rad(-150), // 3
                                    deg2rad(0), deg2rad(110), deg2rad(-150), // 4
                                    deg2rad(0), deg2rad(110), deg2rad(-150); // 5
        // clang-format on
    }
    param.joint_calibration_pos = joint_calibration_pos_T.transpose();

    // TODO: Need to be refined
    param.joint_protect_pos_max << deg2rad(50), deg2rad(112), deg2rad(-30);
    param.joint_protect_pos_min << deg2rad(-50), deg2rad(-30), deg2rad(-234.93);

    return param;
}
