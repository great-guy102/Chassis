/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"

// hal
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

// HW-Components
#include "tick.hpp"

// custom
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"

using hello_world::comm::CanRxMgr;
using hello_world::comm::CanTxMgr;
using hello_world::comm::UartRxMgr;
using hello_world::comm::UartTxMgr;
using hello_world::referee::Referee;
using robot::GimbalChassisComm;
/* Private macro -------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static CanRxMgr *can1_rx_mgr_ptr = nullptr;
static CanTxMgr *can1_tx_mgr_ptr = nullptr;

static CanRxMgr *can2_rx_mgr_ptr = nullptr;
static CanTxMgr *can2_tx_mgr_ptr = nullptr;

static UartRxMgr *rfr_rx_mgr_ptr = nullptr;
static UartTxMgr *rfr_tx_mgr_ptr = nullptr;

static GimbalChassisComm *gc_comm_ptr = nullptr;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void PrivatePointerInit(void);
static void CommAddReceiver(void);
static void CommAddTransmitter(void);
static void CommHardWareInit(void);

/* Exported function definitions ---------------------------------------------*/

void CommTask(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->startTransmit();
  can2_tx_mgr_ptr->startTransmit();
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);

  HW_ASSERT(gc_comm_ptr != nullptr, "gc_comm_ptr is nullptr", gc_comm_ptr);
  if (!gc_comm_ptr->isOffline()) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->errorCallback(hcan);
  can2_tx_mgr_ptr->errorCallback(hcan);
}

uint32_t uart_rx_tick = 0;
uint32_t uart_rx_cb_in = 0;
float uart3_uticks = 0;
float uart6_uticks = 0;
uint32_t uart3_rx_cnt = 0;
uint32_t uart6_rx_cnt = 0;
/**
 * @brief   UART接收回调函数
 * @param   none
 * @retval  none
 * @note    none
 **/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  uart_rx_tick = hello_world::tick::GetTickMs();
  uart_rx_cb_in++;

  // 裁判系统
  if (huart == &huart6) {
    uart6_rx_cnt++;
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim2);
    rfr_rx_mgr_ptr->rxEventCallback(huart, Size);
    uint32_t tick_end = __HAL_TIM_GET_COUNTER(&htim2);
    uart6_uticks = (tick_end - tick_start) / (84.0f * 1e3);
    uart6_rx_cnt--;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // 裁判系统
  if (huart == &huart6) {
    rfr_rx_mgr_ptr->startReceive();
  }
};

/* Private function definitions ----------------------------------------------*/
void CommTaskInit(void) {
  PrivatePointerInit();
  CommAddReceiver();
  CommAddTransmitter();
  CommHardWareInit();
};

static void PrivatePointerInit(void) {
  can1_rx_mgr_ptr = GetCan1RxMgr();
  can1_tx_mgr_ptr = GetCan1TxMgr();

  can2_rx_mgr_ptr = GetCan2RxMgr();
  can2_tx_mgr_ptr = GetCan2TxMgr();

  rfr_rx_mgr_ptr = GetRfrRxMgr();
  rfr_tx_mgr_ptr = GetRfrTxMgr();

  gc_comm_ptr = GetGimbalChassisComm();
}

static void CommAddReceiver(void) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->addReceiver(GetCap()); // TODO调试
  can1_rx_mgr_ptr->addReceiver(GetMotorSteerLeftFront());
  can1_rx_mgr_ptr->addReceiver(GetMotorSteerLeftRear());
  can1_rx_mgr_ptr->addReceiver(GetMotorSteerRightRear());
  can1_rx_mgr_ptr->addReceiver(GetMotorSteerRightFront());

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->addReceiver(GetGimbalChassisComm());
  can2_rx_mgr_ptr->addReceiver(GetMotorWheelLeftFront());
  can2_rx_mgr_ptr->addReceiver(GetMotorWheelLeftRear());
  can2_rx_mgr_ptr->addReceiver(GetMotorWheelRightRear());
  can2_rx_mgr_ptr->addReceiver(GetMotorWheelRightFront());
  can2_rx_mgr_ptr->addReceiver(GetMotorYaw()); // 用于底盘控制，只接受消息

  HW_ASSERT(rfr_rx_mgr_ptr != nullptr, "rfr_rx_mgr_ptr is nullptr",
            rfr_rx_mgr_ptr);
  rfr_rx_mgr_ptr->addReceiver(GetReferee());
}

static void CommAddTransmitter(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  can1_tx_mgr_ptr->addTransmitter(GetCap()); // TODO调试
  can1_tx_mgr_ptr->addTransmitter(GetMotorSteerLeftFront());
  can1_tx_mgr_ptr->addTransmitter(GetMotorSteerLeftRear());
  can1_tx_mgr_ptr->addTransmitter(GetMotorSteerRightRear());
  can1_tx_mgr_ptr->addTransmitter(GetMotorSteerRightFront());

  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can2_tx_mgr_ptr->addTransmitter(GetGimbalChassisComm());
  can2_tx_mgr_ptr->addTransmitter(GetMotorWheelLeftFront());
  can2_tx_mgr_ptr->addTransmitter(GetMotorWheelLeftRear());
  can2_tx_mgr_ptr->addTransmitter(GetMotorWheelRightRear());
  can2_tx_mgr_ptr->addTransmitter(GetMotorWheelRightFront());

  HW_ASSERT(rfr_tx_mgr_ptr != nullptr, "rfr_tx_mgr_ptr is nullptr",
            rfr_tx_mgr_ptr);
  rfr_tx_mgr_ptr->addTransmitter(GetReferee());
}

void CommHardWareInit(void) {
  // CAN init
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->filterInit();
  can1_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan1);

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->filterInit();
  can2_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan2);

  // rfr DMA Init
  HW_ASSERT(rfr_rx_mgr_ptr != nullptr, "rfr_rx_mgr_ptr is nullptr",
            rfr_rx_mgr_ptr);
  rfr_rx_mgr_ptr->startReceive();
};