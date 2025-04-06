/**
 *******************************************************************************
 * @file      :ins_rfr.hpp
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
#ifndef INS_RFR_HPP_
#define INS_RFR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hello_world::referee::RobotPerformancePackage *GetRobotPerformancePackage();
hello_world::referee::RobotPowerHeatPackage *GetRobotPowerHeatPackage();
hello_world::referee::RobotShooterPackage *GetRobotShooterPackage();
hello_world::referee::RobotHurtPackage *GetRobotHurtPackage();
hello_world::referee::Referee *GetReferee();

#endif /* INS_RFR_HPP_ */
