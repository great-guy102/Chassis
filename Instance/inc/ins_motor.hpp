/**
 *******************************************************************************
 * @file      :ins_motor.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_MOTOR_HPP_
#define CHASSIS_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"

namespace hw_motor = hello_world::motor;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
// 创建轮电机
hw_motor::Motor *GetMotorWheelLeftFront();
hw_motor::Motor *GetMotorWheelLeftRear();
hw_motor::Motor *GetMotorWheelRightRear();
hw_motor::Motor *GetMotorWheelRightFront();
// 创建舵电机
hw_motor::Motor *GetMotorSteerLeftFront();
hw_motor::Motor *GetMotorSteerLeftRear();
hw_motor::Motor *GetMotorSteerRightRear();
hw_motor::Motor *GetMotorSteerRightFront();
// 创建yaw电机
hw_motor::Motor *GetMotorYaw();

#endif /* CHASSIS_INSTANCE_INS_MOTOR_HPP_ */
