/**
 * @file basic_controller.h
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-11-30
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once
//#include "common/utilities/pin_header.h"
/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <thread>

/* external project header files */
#include <Eigen/Dense>
//#include <ros/ros.h>
//#include <sensor_msgs/Joy.h>

/* internal project header files */
//#include "common/leg_controllers/gait_scheduler.h"
#include "basic_controller.h"
#include "control_state.h"

//#include "common/models/leg_kinematic.h"
#include "robot_state.h"

// #include "common/leg_controllers/bezier.h"
// #include "common/leg_controllers/cycloid.h"
// #include "common/leg_controllers/polynomial.h"
// #include "common/leg_controllers/trajectory_generator.h"
#include "timer.h"

enum class CommandType
{
    kParam,     // only update parameters
    kFootForce, // foot force commands
    kFootPV,    // foot position velocity hybrid & joint torque feedforward commands
    kFoot,      // foot position velocity hybrid & foot torque feedforward commands
    kJoint,     // joint commands
};

#define LEG(i) (1 << (i))
#define ALL_LEG (LEG(0) | LEG(1) | LEG(2) | LEG(3) | LEG(4) | LEG(5))

class BasicController
{
  public:
    BasicController(ros::NodeHandle &r_nh,RobotState &r_robot_state, ControlState &r_control_state);
    virtual ~BasicController() = default;
    void controllerThread1(void);
    void controllerThread2(void);
    void initMotor(void);  
    int thread_num{};

  protected:
    // virtual void rawControllerThread1(void) = 0;
    // virtual void rawControllerThread2(void); // not required, so not a pure virtual function
    // virtual void controlModeDeal(double dt) = 0;
    // virtual void controlLoop(double dt) = 0;
    // virtual void joystickDeal(void) = 0;
    // virtual void getParamFromYaml(void) = 0;

    // virtual void feedbackHook(void){};
    // virtual void commandHook(void){};

    void threadConfig(void);

    bool moveFootPosition(CommandType input_cmd_type, CommandType output_cmd_type,
                          const Eigen::MatrixXd &input_cmd, double interpolation_sum_t, double dt);
    void updateCommand(CommandType command_type, double dt, uint8_t leg_num,
                       bool use_joint_vel_cmd = true);
    void getFeedback(void);
    
    bool isMoving(uint8_t leg_num);
    bool isArrived(uint8_t leg_num);

    // ros
    ros::NodeHandle &nh_;

    // robot info
    RobotState &robot_state_;
    //Kinematic kinematic_;

    // control related
    //GaitScheduler gait_scheduler_;
    //std::unique_ptr<TrajectoryGenerator> trajectory_generator_{};
    ControlState &control_state_;
    Timer timer_;
};
