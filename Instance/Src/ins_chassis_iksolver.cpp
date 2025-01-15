/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
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
#include "ins_chassis_iksolver.hpp"
/* Private constants ---------------------------------------------------------*/

const float kWheelRadius = 66 * 0.001; ///< 轮子半径 [m]
const float kWheelBase = 0.334;        ///< 左右轮距 [m]
const float kWheelTrack = 0.334;       ///< 前后轮距 [m]

const hw_chassis_iksolver::PosVec kCenterPos =
    hw_chassis_iksolver::PosVec(0, 0);
const hw_chassis_iksolver::WheelParams kIkSolverParams = {
    .opt_mask =
        hw_chassis_iksolver::SteeredStandardWheel::kOptMaskCosRotSpd |
        hw_chassis_iksolver::SteeredStandardWheel::kOptMaskMinThetaVelDelta,
    .radius = kWheelRadius,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_chassis_iksolver::ChassisIkSolver unique_chassis_iksolver =
    hw_chassis_iksolver::ChassisIkSolver(kCenterPos);
bool is_chassis_iksolver_init = false;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void InitChassisIkSolver(void) {
  hw_chassis_iksolver::WheelParams ik_solver_params[4] = {
      kIkSolverParams,
      kIkSolverParams,
      kIkSolverParams,
      kIkSolverParams,
  };

  // 左前轮
  ik_solver_params[0].wheel_pos =
      hw_chassis_iksolver::PosVec(kWheelTrack / 2, kWheelBase / 2);

  // 左后轮
  ik_solver_params[1].wheel_pos =
      hw_chassis_iksolver::PosVec(-kWheelTrack / 2, kWheelBase / 2);

  // 右后轮
  ik_solver_params[2].wheel_pos =
      hw_chassis_iksolver::PosVec(-kWheelTrack / 2, -kWheelBase / 2);

  // 右前轮
  ik_solver_params[3].wheel_pos =
      hw_chassis_iksolver::PosVec(kWheelTrack / 2, -kWheelBase / 2);

  for (int i = 0; i < 4; i++) {
    unique_chassis_iksolver.append(
        hw_chassis_iksolver::WheelType::kSteeredStandard, ik_solver_params[i]);
  }
}

/* Exported function definitions ---------------------------------------------*/
hw_chassis_iksolver::ChassisIkSolver *GetChassisIkSolver(void) {
  if (!is_chassis_iksolver_init) {
    is_chassis_iksolver_init = true;
    InitChassisIkSolver();
  }
  return &unique_chassis_iksolver;
};
/* Private function definitions ----------------------------------------------*/