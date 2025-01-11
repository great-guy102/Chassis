/**
 *******************************************************************************
 * @file      :ins_imu.cpp
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
#include "ins_imu.hpp"
#include "spi.h"

/* Private constants ---------------------------------------------------------*/
static const hw_imu::ImuConfig kImuConfig = {
    .rot_mat_flatten = {1, 0, 0, 0, 1, 0, 0, 0, 1}, // TODO: 旋转矩阵待标定
    .bmi088_hw_config =
        {
            // C板
            .hspi = &hspi1,
            .acc_cs_port = GPIOA,
            .acc_cs_pin = GPIO_PIN_4,
            .gyro_cs_port = GPIOB,
            .gyro_cs_pin = GPIO_PIN_0,
        },
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_imu::Imu *GetImu(void) {
  static hw_imu::Imu unique_imu = hello_world::imu::Imu(kImuConfig);
  return &unique_imu;
}

/* Private function definitions ----------------------------------------------*/
