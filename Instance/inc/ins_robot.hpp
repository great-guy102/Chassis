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
#ifndef CHASSIS_INSTANCE_INS_ROBOT_HPP_
#define CHASSIS_INSTANCE_INS_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
#include "robot.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

robot::Chassis *GetChassis();
robot::Robot *GetRobot();
/* Exported function prototypes ----------------------------------------------*/

#endif /* CHASSIS_INSTANCE_INS_ROBOT_HPP_ */
