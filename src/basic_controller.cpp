/**
 * @file basic_controller.cpp
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-11-30
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "basic_controller.h"

/* c system header files */

/* c++ standard library header files */
#include <memory>
#include <mutex>
#include <shared_mutex>

/* external project header files */

/* internal project header files */
//#include "common/utilities/basic_func.hpp"
#include "basic_math.hpp"

/**
 * @brief Construct a new Basic Controller:: Basic Controller object
 * @param[in] r_nh the reference to ros node handle
 * @param[in] r_robot_state the reference to robot state
 * @param[in] r_control_state the reference to control state
 */
BasicController::BasicController(ros::NodeHandle &r_nh, RobotState &r_robot_state,
    ControlState &r_control_state)
: nh_(r_nh), robot_state_(r_robot_state), control_state_(r_control_state),
timer_(TimerSourceType::kRos) 
{
    std::cout << "BasicController initialized with RobotState and ControlState." << std::endl;

    // 检查 RobotState 参数的有效性
    if (robot_state_.param.leg_num <= 0 || robot_state_.param.leg_dof <= 0)
    {
        throw std::invalid_argument("Invalid RobotState parameters: leg_num or leg_dof is not valid.");
    }

    // 初始化控制参数
    control_state_.motor_init_torque.setZero();
    control_state_.joint.kp.setZero();
    control_state_.joint.kd.setZero();
}

/**
 * @brief initialize motor
 */
void BasicController::initMotor(void)
{
    const int kLegNum = robot_state_.param.leg_num;
    const int kLegDof = robot_state_.param.leg_dof;

    static int joint_num = 0;
    // TODO: add a first-in flag to solve this problem completely?
    // no motor in hardware or in gazebo, motor feedback equal to zero
    // we need to ensure that last_pos not equal to now_pos at the beginning
    static Eigen::MatrixXd last_pos = Eigen::MatrixXd::Random(kLegDof, kLegNum);
    Eigen::MatrixXd current_pos(kLegDof, kLegNum);
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> init_finished_flag(kLegDof, kLegNum);

    double time_compensate = 0;
    int sum_flag = 0;

    switch (robot_state_.param.motor_init_type)
    {
    case MotorInitType::kAbsoluteEncoder:
        robot_state_.joint.pos_zero.setZero();
        robot_state_.motor_init_finished_flag = true;
        break;
    case MotorInitType::kManualCalibration:
        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();
        robot_state_.joint.pos_zero = current_pos;
        robot_state_.motor_init_finished_flag = true;
        break;
    // TODO: rewrite this part(Hexapod specific)
    case MotorInitType::kLyingAutoCalibration:
        timer_.start();

        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();

        init_finished_flag.setZero();

        // determine whether the joint has reached the mechanical limit
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.02)
            {
                init_finished_flag(joint_num, j) = true;
            }
        }

        //  determine whether all six joints in the same position reached the mechanical limit
        sum_flag = 0;
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            sum_flag += init_finished_flag(joint_num, j);
        }

        if (sum_flag == 6)
        {
            // save motor zero position
            robot_state_.cmd_mutex.lock();
            robot_state_.joint.pos_zero.row(joint_num) = current_pos.row(joint_num);
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                // record the zero position
                robot_state_.joint.pos_cmd(joint_num, j) =
                    robot_state_.param.joint_calibration_pos(joint_num, j);
                // clear torque command and set kp, use PD to keep position
                robot_state_.joint.torque_cmd(joint_num, j) = 0;
                control_state_.joint.kp(joint_num, j) = control_state_.joint.kp_default(joint_num);
                robot_state_.joint.kp(joint_num, j) = control_state_.joint.kp(joint_num, j);
            }
            robot_state_.cmd_mutex.unlock();

            //ROS_DEBUG_STREAM("joint_pos_zero: " << (current_pos.row(joint_num)));
            std::cout << "Joint " << joint_num << " zero position: "
                      << current_pos.row(joint_num) << std::endl;
            // turn to the next joints
            ++joint_num;
        }

        if (joint_num == 3)
        {
            robot_state_.motor_init_finished_flag = true;
        }
        else
        {
            robot_state_.cmd_mutex.lock();
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                if (joint_num > 0)
                    robot_state_.joint.torque_cmd(joint_num, j) =
                        control_state_.motor_init_torque(joint_num);
            }
            robot_state_.cmd_mutex.unlock();
        }

        last_pos = current_pos;

        timer_.stop();

        // maintain a fixed control frequency
        time_compensate =
            1000. / (double)control_state_.motor_init_frequency - timer_.elapsedMilliseconds();
        if (!std::isfinite(time_compensate))
        {
            // ROS_FATAL("error frequency");
            // basic_func::exitProgram();
            std::cerr << "Error: Invalid frequency compensation value." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (time_compensate > 0)
        {
            // use ros interface to support simulated time
            //ros::Duration(time_compensate / 1000.).sleep();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(time_compensate)));
        }
        break;
    case MotorInitType::kAutoCalibration:
        timer_.start();

        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();

        init_finished_flag.setZero();

        // determine whether the joint has reached the mechanical limit
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.01)
            {
                init_finished_flag(joint_num, j) = true;
            }
        }

        //  determine whether all six joints in the same position reached the mechanical limit
        sum_flag = 0;
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            sum_flag += init_finished_flag(joint_num, j);
        }

        if (sum_flag == 6)
        {
            // save motor zero position
            robot_state_.cmd_mutex.lock();
            robot_state_.joint.pos_zero.row(joint_num) = current_pos.row(joint_num);
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                // record the zero position
                robot_state_.joint.pos_cmd(joint_num, j) =
                    robot_state_.param.joint_calibration_pos(joint_num, j);
                // clear torque command and set kp, use PD to keep position
                robot_state_.joint.torque_cmd(joint_num, j) = 0;
                control_state_.joint.kp(joint_num, j) = control_state_.joint.kp_default(joint_num);
                robot_state_.joint.kp(joint_num, j) = control_state_.joint.kp(joint_num, j);
            }
            robot_state_.cmd_mutex.unlock();

            //ROS_DEBUG_STREAM("joint_pos_zero: " << (current_pos.row(joint_num)));
            std::cout << "Joint " << joint_num << " zero position: "
                  << current_pos.row(joint_num) << std::endl;
            // turn to the next joints
            ++joint_num;
        }

        if (joint_num == 3)
        {
            robot_state_.motor_init_finished_flag = true;
        }
        else
        {
            // set the next joints' commands, then delay and wait for motor running
            robot_state_.cmd_mutex.lock();
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                robot_state_.joint.torque_cmd(joint_num, j) =
                    control_state_.motor_init_torque(joint_num);
            }
            robot_state_.cmd_mutex.unlock();
        }

        last_pos = current_pos;

        timer_.stop();

        // maintain a fixed control frequency
        time_compensate =
            1000. / (double)control_state_.motor_init_frequency - timer_.elapsedMilliseconds();
        if (!std::isfinite(time_compensate))
        {
            // ROS_FATAL("error frequency");
            // basic_func::exitProgram();
            std::cerr << "Error: Invalid frequency compensation value." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (time_compensate > 0)
        {
            // use ros interface to support simulated time
            //ros::Duration(time_compensate / 1000.).sleep();
            std::this_thread::sleep_for(std::chrono::microseconds(int(1000 * time_compensate)));
        }
        break;
    }
}

/**
 * @brief call feedbackHook() to get feedback
 */
// void BasicController::getFeedback(void)
// {
//     std::unique_lock<std::shared_mutex> lock(robot_state_.fdb_mutex);
//     feedbackHook();

//     // used in kStop mode
//     control_state_.joint.pos_fdb = robot_state_.joint.pos_fdb;
// }