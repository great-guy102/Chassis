/** 
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private constants ---------------------------------------------------------*/
//轮电机通用参数配置
const hw_motor::OptionalParams kWheelMotorParams = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
    .max_raw_input_lim = 16384.0,                     // 报文输入限制
    .max_curr_input_lim = 20.0,                      // 电流输入限制为3.0A
};

//舵电机通用参数配置
hw_motor::OptionalParams kSteerMotorParams = {
    .input_type = hw_motor::InputType::kCurr,             // 输入类型为电流控制
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,  // 角度范围为[-π, π)
    .dir = hw_motor::kDirRev,                          // 方向为正向
    .remove_build_in_reducer = false,               // 不移除内置减速器
    .ex_redu_rat = 1,                               // 外置减速器的减速比为1
    .max_raw_input_lim = 16384.0,                     // 报文输入限制
    .max_curr_input_lim = 3.0,                      // 电流输入限制为3.0A
};

// //舵电机零位设置
// const float steer_motor_offset[4] = {2.09644055, 
//                                     -3.11436129, 
//                                     -3.13814092, 
//                                     -1.03863168};

//白车
const float steer_motor_offset[4] = {-0.52813754,
                                     -2.06844183,
                                      0.00728712,
                                      1.58517917};

const hw_motor::OptionalParams kYawMotorParams = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 1.53280616,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};


// TODO: 这里的 MotorId 需要按照实际情况修改

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
//轮电机单例
static bool is_motor_wheel_left_front_inited = false;
hw_motor::M3508 unique_motor_wheel_left_front;

static bool is_motor_wheel_left_rear_inited = false;
hw_motor::M3508 unique_motor_wheel_left_rear;

static bool is_motor_wheel_right_rear_inited = false;
hw_motor::M3508 unique_motor_wheel_right_rear;

static bool is_motor_wheel_right_front_inited = false;
hw_motor::M3508 unique_motor_wheel_right_front;

//舵电机单例
static bool is_motor_steer_left_front_inited = false;
hw_motor::GM6020 unique_motor_steer_left_front;

static bool is_motor_steer_left_rear_inited = false;
hw_motor::GM6020 unique_motor_steer_left_rear;

static bool is_motor_steer_right_rear_inited = false;
hw_motor::GM6020 unique_motor_steer_right_rear;

static bool is_motor_steer_right_front_inited = false;
hw_motor::GM6020 unique_motor_steer_right_front;

//yaw电机单例
static bool is_motor_yaw_inited = false;
hw_motor::DM_J4310 unique_motor_yaw;

hw_motor::Motor* GetMotorWheelLeftFront() { 
    if(!is_motor_wheel_left_front_inited){
        unique_motor_wheel_left_front = hw_motor::M3508(0x01, kWheelMotorParams);
        is_motor_wheel_left_front_inited = true;
    }
    return &unique_motor_wheel_left_front; 
};

hw_motor::Motor* GetMotorWheelLeftRear() { 
    if(!is_motor_wheel_left_rear_inited){
        unique_motor_wheel_left_rear = hw_motor::M3508(0x02, kWheelMotorParams);
        is_motor_wheel_left_rear_inited = true;
    }
    return &unique_motor_wheel_left_rear; 
};

hw_motor::Motor* GetMotorWheelRightRear() { 
    if(!is_motor_wheel_right_rear_inited){
        unique_motor_wheel_right_rear = hw_motor::M3508(0x03, kWheelMotorParams);
        is_motor_wheel_right_rear_inited = true;
    }
    return &unique_motor_wheel_right_rear; 
};

hw_motor::Motor* GetMotorWheelRightFront() { 
    if(!is_motor_wheel_right_front_inited){
        unique_motor_wheel_right_front = hw_motor::M3508(0x04, kWheelMotorParams);
        is_motor_wheel_right_front_inited = true;
    }
    return &unique_motor_wheel_right_front; 
};

hw_motor::Motor* GetMotorSteerLeftFront() { 
    if(!is_motor_steer_left_front_inited){
        hw_motor::OptionalParams SteerMotorParamsLeftFront = kSteerMotorParams;
        SteerMotorParamsLeftFront.angle_offset = steer_motor_offset[0];
        unique_motor_steer_left_front = hw_motor::GM6020(0x01, SteerMotorParamsLeftFront);
        is_motor_steer_left_front_inited = true;
    }
    return &unique_motor_steer_left_front; 
};

hw_motor::Motor* GetMotorSteerLeftRear() { 
    if(!is_motor_steer_left_rear_inited){
        hw_motor::OptionalParams SteerMotorParamsLeftRear = kSteerMotorParams;
        SteerMotorParamsLeftRear.angle_offset = steer_motor_offset[1];
        unique_motor_steer_left_rear = hw_motor::GM6020(0x02, SteerMotorParamsLeftRear);
        is_motor_steer_left_rear_inited = true;
    }
    return &unique_motor_steer_left_rear; 
};

hw_motor::Motor* GetMotorSteerRightRear() { 
    if(!is_motor_steer_right_rear_inited){
        hw_motor::OptionalParams SteerMotorParamsRightRear = kSteerMotorParams;
        SteerMotorParamsRightRear.angle_offset = steer_motor_offset[2];
        unique_motor_steer_right_rear = hw_motor::GM6020(0x03, SteerMotorParamsRightRear);
        is_motor_steer_right_rear_inited = true;
    }
    return &unique_motor_steer_right_rear; 
};

hw_motor::Motor* GetMotorSteerRightFront() { 
    if(!is_motor_steer_right_front_inited){
        hw_motor::OptionalParams SteerMotorParamsRightFront = kSteerMotorParams;
        SteerMotorParamsRightFront.angle_offset = steer_motor_offset[3];
        unique_motor_steer_right_front = hw_motor::GM6020(0x04, SteerMotorParamsRightFront);
        is_motor_steer_right_front_inited = true;
    }
    return &unique_motor_steer_right_front; 
};

hw_motor::Motor* GetMotorYaw() { 
    if(!is_motor_yaw_inited){
        unique_motor_yaw = hw_motor::DM_J4310(0x01, kYawMotorParams);
        is_motor_yaw_inited = true;
    }
    return &unique_motor_yaw;
};

/* Private function definitions ----------------------------------------------*/