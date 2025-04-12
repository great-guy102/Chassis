/**
 *******************************************************************************
 * @file      :ins_robot.hpp
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
#ifndef INSTANCE_INS_ROBOT_HPP_
#define INSTANCE_INS_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
#include "feed.hpp"
#include "gimbal.hpp"
#include "robot.hpp"
#include "shooter.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

robot::Chassis *GetChassis();
robot::Gimbal *GetGimbal();
robot::Shooter *GetShooter();
robot::Robot *GetRobot();
/* Exported function prototypes ----------------------------------------------*/

#endif /* INSTANCE_INS_ROBOT_HPP_ */
