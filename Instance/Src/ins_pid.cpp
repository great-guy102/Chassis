/** 
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutWheel = 16384.0f;  ///< 3508电流控制的最大输出
const float kMaxPidOutSteerAngle = 38.0f;  ///< 6020电流控制的角度环限幅
const float kMaxPidOutSteerVel = 30000.0f;  ///< 速度环限幅
const float kMaxPidOutFollowOmega = 35.0f;

const hw_pid::OutLimit kOutLimitWheel = hw_pid::OutLimit(true, -kMaxPidOutWheel, kMaxPidOutWheel);
const hw_pid::OutLimit kOutLimitSteerAngle = hw_pid::OutLimit(true, -kMaxPidOutSteerAngle, kMaxPidOutSteerAngle);
const hw_pid::OutLimit kOutLimitSteerVel = hw_pid::OutLimit(true, -kMaxPidOutSteerVel, kMaxPidOutSteerVel);
const hw_pid::OutLimit kOutLimitFollowOmega = hw_pid::OutLimit(true, -kMaxPidOutFollowOmega, kMaxPidOutFollowOmega);

const hw_pid::MultiNodesPid::ParamsList kPidParamsWheel = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 1500.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -10000.0, 10000.0),
        .out_limit = kOutLimitWheel,
     },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsSteer = {
    {
        .auto_reset = false,  ///< 是否自动清零
        .kp = 120.0f,         // 16
        .ki = 0.0f,
        .kd = 0.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI),
        .out_limit = kOutLimitSteerAngle,
    },
    {
        .auto_reset = false,  ///< 是否自动清零
        .kp = 600.0f,          // 1200.0f,
        .ki = 0.0f,          // 3.5
        .kd = 0.0f,          // 100
        .setpoint_ramping = hw_pid::SetpointRamping(true, -10, 10, 0.95),
        .out_limit = kOutLimitSteerVel,
    }
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowOmega = {
    {
     .auto_reset = true,
     .kp = 6.0,
     .ki = 0,
     .kd = 5.0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .period_sub = hw_pid::PeriodSub(true, 2 * PI),
     .inte_anti_windup = hw_pid::InteAntiWindup(true, -10.00, 10.00),
     .out_limit = kOutLimitFollowOmega,
     },
    //  这个双环不一定有用，拿的是YAW轴电机速度
    // 这玩意儿云台动起来就有额外偏差，有点问题，建议注释掉只调单环
    // {
    //  .auto_reset = true,
    //  .kp = 3,
    //  .ki = 0,
    //  .kd = 0,
    //  .period_sub = hw_pid::PeriodSub(false, 0),
    //  .out_limit = hw_pid::OutLimit(true, -80, 80),
    //  },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//轮电机PID
hw_pid::MultiNodesPid unique_pid_wheel_left_front(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_left_rear(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_right_rear(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);
hw_pid::MultiNodesPid unique_pid_wheel_right_front(kPidTypeCascade, kOutLimitWheel, kPidParamsWheel);

//舵电机PID
hw_pid::MultiNodesPid unique_pid_steer_left_front(kPidTypeCascade, kOutLimitSteerVel, kPidParamsSteer);
hw_pid::MultiNodesPid unique_pid_steer_left_rear(kPidTypeCascade, kOutLimitSteerVel, kPidParamsSteer);
hw_pid::MultiNodesPid unique_pid_steer_right_rear(kPidTypeCascade, kOutLimitSteerVel, kPidParamsSteer);
hw_pid::MultiNodesPid unique_pid_steer_right_front(kPidTypeCascade, kOutLimitSteerVel, kPidParamsSteer);

//自旋PID
hw_pid::MultiNodesPid unique_pid_follow_omega(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowOmega);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* GetPidMotorWheelLeftFront() { return &unique_pid_wheel_left_front; };
hw_pid::MultiNodesPid* GetPidMotorWheelLeftRear() { return &unique_pid_wheel_left_rear; };
hw_pid::MultiNodesPid* GetPidMotorWheelRightRear() { return &unique_pid_wheel_right_rear; };
hw_pid::MultiNodesPid* GetPidMotorWheelRightFront() { return &unique_pid_wheel_right_front; };

hw_pid::MultiNodesPid* GetPidMotorSteerLeftFront() { return &unique_pid_steer_left_front; };
hw_pid::MultiNodesPid* GetPidMotorSteerLeftRear() { return &unique_pid_steer_left_rear; };
hw_pid::MultiNodesPid* GetPidMotorSteerRightRear() { return &unique_pid_steer_right_rear; };
hw_pid::MultiNodesPid* GetPidMotorSteerRightFront() { return &unique_pid_steer_right_front; };

hw_pid::MultiNodesPid* GetPidFollowOmega() { return &unique_pid_follow_omega; };
/* Private function definitions ----------------------------------------------*/
