#include <ctime>
#include <chrono>



enum class TimerSourceType
{
    kNone,
    kRos,
    kSystem
};

class Timer
{
  public:
    explicit Timer(TimerSourceType source_type) : source_type_(source_type){};
    ~Timer() = default;
    void start(void);
    void stop(void);
    [[nodiscard]] double elapsedMilliseconds(void) const;
    [[nodiscard]] double elapsedSeconds(void) const;

  private:
    TimerSourceType source_type_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> sys_start_time_, sys_stop_time_;
    bool running_flag_{};
};




Timer timer_;


int maxK = 100;
switch (calibrationStep)
{
    case -1: // 给定力控命令
        forceControlTime++;
        robot_state_.cmd_mutex.lock();
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            control_state_.joint.kp(joint_num, j) = 0;
            robot_state_.joint.kp(joint_num, j) = 0;
            if (joint_num == 2) // there exit difference between on-board calibration and lying calibration
            {
                robot_state_.joint.torque_cmd(joint_num, j) =
                    control_state_.motor_init_torque(joint_num);
            }
        }
        robot_state_.cmd_mutex.unlock();
        if(forceControlTime > 2)
        {
            calibrationStep = 0;
            forceControlTime = 0;
        }
    break;
    
    case 0:  // 判断是否到达机械限位
        init_finished_flag.setZero();
        // determine whether the joint has reached the mechanical limit
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.002)
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
            ROS_DEBUG_STREAM("joint_pos_zero: " << (current_pos.row(joint_num)));
            // turn to the next joints
            ++joint_num;
            if (joint_num == 1) // 力控阶段结束,进入根关节回0度case
            {
                calibrationStep = 1;
            }
            else if(joint_num == 2) // 膝关节继续力控
            {
                calibrationStep = -1;
            }
            else if(joint_num == 3) // 所有关节力控矫正结束，开始控制关节回到指定角度
            {
                calibrationStep = 2;
            }
        }
    break;
    case 1: // 根关节回0度位置
        control_state_.motor_init_frequency = 100;
        kk++;
        if(kk > maxK) kk = maxK;
        robot_state_.cmd_mutex.lock();
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            float targetAngle = 0.0f;
            float tmpTgt = robot_state_.param.joint_calibration_pos(0, j)
                +(targetAngle - robot_state_.param.joint_calibration_pos(0, j))/float(maxK) * kk;
            robot_state_.joint.pos_cmd(0, j) = tmpTgt;
        }
        robot_state_.cmd_mutex.unlock();
        if (kk == maxK)
        {
            control_state_.motor_init_frequency = 1;
            calibrationStep = -1;  // 回到力控阶段
        }
    break;
    case 2:
        control_state_.motor_init_frequency = 100;
        gg++;
        if(gg > maxK) gg = maxK;
        robot_state_.cmd_mutex.lock();
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            float targetAngle = -90.0f/180.0*3.1415926f;
            float tmpTgt = robot_state_.param.joint_calibration_pos(2, j)
                +(targetAngle - robot_state_.param.joint_calibration_pos(2, j))/float(maxK) * gg;
            robot_state_.joint.pos_cmd(2, j) = tmpTgt;
            targetAngle = -0.0f/180.0*3.1415926f;
            tmpTgt = robot_state_.param.joint_calibration_pos(1, j)
                +(targetAngle - robot_state_.param.joint_calibration_pos(1, j))/float(maxK) * gg;
            robot_state_.joint.pos_cmd(1, j) = tmpTgt;
        }
        robot_state_.cmd_mutex.unlock();
        if (gg == maxK)
        {
            control_state_.motor_init_frequency = 1;
            calibrationStep = 99;
        }
    break;
    case 99:
        kk = 0;
        calibrationStep = -1;
        gg = 0;
        joint_num = 0;
        robot_state_.cmd_mutex.lock();
        robot_state_.motor_init_finished_flag = true;
        robot_state_.cmd_mutex.unlock();
        std::cout << "calibration success" << std::endl;
    break;
}
last_pos = current_pos;
timer_.stop();
// maintain a fixed control frequency
time_compensate =
    1000. / (double)control_state_.motor_init_frequency - timer_.elapsedMilliseconds();
if (!isfinite(time_compensate))
{
    ROS_FATAL("error frequency");
    basic_func::exitProgram();
}
if (time_compensate > 0)
{
    // use ros interface to support simulated time
    ros::Duration(time_compensate / 1000.).sleep();
    // std::this_thread::sleep_for(std::chrono::microseconds(int(1000 * time_compensate)));
}