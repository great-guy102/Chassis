/**
 *******************************************************************************
 * @file      :ins_rfr.cpp
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
#include "ins_rfr.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::referee::InterKbCtrlerPackage unique_inter_kb_ctrler_package;
hello_world::referee::RobotPerformancePackage unique_robot_performance_package;
hello_world::referee::RobotPowerHeatPackage unique_robot_power_heat_package;
hello_world::referee::RobotShooterPackage unique_robot_shooter_package;
hello_world::referee::Referee unique_referee;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/
hello_world::referee::InterKbCtrlerPackage *GetInterKbCtrlerPackage() {
  return &unique_inter_kb_ctrler_package;
};
hello_world::referee::RobotPerformancePackage *GetRobotPerformancePackage() {
  return &unique_robot_performance_package;
};
hello_world::referee::RobotPowerHeatPackage *GetRobotPowerHeatPackage() {
  return &unique_robot_power_heat_package;
};
hello_world::referee::RobotShooterPackage *GetRobotShooterPackage() {
  return &unique_robot_shooter_package;
};
hello_world::referee::Referee *GetReferee(void) {
  unique_referee.appendRxPkg(GetInterKbCtrlerPackage());
  unique_referee.appendRxPkg(GetRobotPerformancePackage());
  unique_referee.appendRxPkg(GetRobotPowerHeatPackage());
  unique_referee.appendRxPkg(GetRobotShooterPackage());
  return &unique_referee;
};

/* Private function definitions ----------------------------------------------*/
