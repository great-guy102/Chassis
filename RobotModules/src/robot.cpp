/**
 *******************************************************************************
 * @file      :robot.cpp
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
#include "robot.hpp"
#include "usart.h"
/* Private macro -------------------------------------------------------------*/

namespace robot {
/* Private constants ---------------------------------------------------------*/
const hello_world::referee::RobotPerformanceData kDefaultRobotPerformanceData = {
    .robot_id =
        static_cast<uint8_t>(Robot::RobotId::kBlueStandard3), ///< 本机器人ID

    .robot_level = 0,  ///< 机器人等级
    .current_hp = 200, ///< 机器人当前血量
    .maximum_hp = 200, ///< 机器人血量上限

    .shooter_barrel_cooling_value = 40, ///< 机器人枪口热量每秒冷却值
    .shooter_barrel_heat_limit = 100,   ///< 机器人枪口热量上限

    .chassis_power_limit =
        55, ///< 底盘功率限制，若不选择底盘或发射机构类型，则在七分钟比赛阶段开始后，未选择的底盘性能类型将被默认选择为“血量优先”

    .power_management_gimbal_output =
        1, ///< 电源管理模块 gimbal 口输出：0-关闭，1-开启
    .power_management_chassis_output =
        1, ///< 电源管理模块 chassis 口输出：0-关闭，1-开启
    .power_management_shooter_output =
        1, ///< 电源管理模块 shooter 口输出：0-关闭，1-开启
}; // namespace robot
const hello_world::referee::RobotPowerHeatData kDefaultRobotPowerHeatData = {
    .buffer_energy = 60, ///< 缓冲能量，单位：J

    .shooter_17mm_1_barrel_heat = 0, ///< 第一个17mm发射机构的枪口热量
    .shooter_17mm_2_barrel_heat = 0, ///< 第二个17mm发射机构的枪口热量
    .shooter_42mm_barrel_heat = 0,   ///< 42mm发射机构的枪口热量
};
const hello_world::referee::RobotShooterData kDefaultRobotShooterData = {
    .bullet_type = static_cast<uint8_t>(
        hello_world::referee::BulletType::k17mm), ///< 弹丸类型，@see
                                                  ///< BulletType

    .shooter_id = static_cast<uint8_t>(
        hello_world::referee::ShooterId::k17mm1), ///< 发射机构ID，@see
                                                  ///< ShooterId

    .launching_frequency = 0, ///< 弹丸射频，单位：Hz
    .bullet_speed = 22.0f,    ///< 弹丸初速度，单位：m/s
};
const hello_world::referee::RobotHurtData kDefaultRobotHurtData = {
    .module_id = 0, ///< 模块ID
    .hp_deduction_reason = static_cast<uint8_t>(
        hello_world::referee::HpDeductionReason::
            kArmorHit), ///< 扣血原因，@see HpDeductionReason
};
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
// 状态机主要接口函数
void Robot::update() {
  updateData();
  updatePwrState();
};

void Robot::updateData() {
  updateWorkTick();
  updateImuData();
  updateRfrData();
};

void Robot::updateImuData() {
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isOffsetCalcFinished()) {
    is_imu_caled_offset_ = true;
  }
};

void Robot::updateRfrData() {
  HW_ASSERT(referee_ptr_ != nullptr, "RFR pointer is null", referee_ptr_);
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);

  PerformancePkg::Data rpp_data = kDefaultRobotPerformanceData;
  PowerHeatPkg::Data rph_data = kDefaultRobotPowerHeatData;
  ShooterPkg::Data rsp_data = kDefaultRobotShooterData;
  HurtPkg::Data rhp_data = kDefaultRobotHurtData;
  static uint8_t rfr_bullet_shot_cnt = 0;

  if (!referee_ptr_->isOffline()) {
    rpp_data = rfr_performance_pkg_ptr_->getData();
    rph_data = rfr_power_heat_pkg_ptr_->getData();
    rsp_data = rfr_shooter_pkg_ptr_->getData();
    rhp_data = rfr_hurt_pkg_ptr_->getData();

    if (!referee_ptr_->isOffline()) {
      if (!rfr_shooter_pkg_ptr_->isHandled()) {
        rfr_shooter_pkg_ptr_->setHandled();
        rfr_bullet_shot_cnt = (rfr_bullet_shot_cnt == 0 ? 1 : 0);
      }
    }
  }

  if (!referee_ptr_->isOffline()) {
    robot_id_ = (RobotId)rpp_data.robot_id;
  } else {
    robot_id_ = Robot::RobotId::kRedStandard3;
  }

  // Chassis
  Chassis::RfrData chassis_rfr_data;
  chassis_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
  chassis_rfr_data.is_pwr_on = rpp_data.power_management_chassis_output;
  chassis_rfr_data.pwr_limit = rpp_data.chassis_power_limit;
  chassis_rfr_data.pwr_buffer = rph_data.buffer_energy;
  chassis_rfr_data.current_hp = rpp_data.current_hp;
  chassis_ptr_->setRfrData(chassis_rfr_data);

  // Gimbal
  GimbalChassisComm::RefereeData::ChassisPart &gimbal_rfr_data =
      gc_comm_ptr_->referee_data().cp;
  gimbal_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
  gimbal_rfr_data.is_rfr_shooter_power_on =
      rpp_data.power_management_shooter_output;
  gimbal_rfr_data.is_rfr_gimbal_power_on =
      rpp_data.power_management_gimbal_output;
  gimbal_rfr_data.rfr_bullet_shot_cnt =
      rfr_bullet_shot_cnt; // is_new_bullet_shot
  gimbal_rfr_data.robot_id = (hello_world::referee::RfrId)rpp_data.robot_id;

  // Shooter
  //  TODO: 换成实际的枪口
  gimbal_rfr_data.bullet_speed = rsp_data.bullet_speed;
  gimbal_rfr_data.shooter_heat = rph_data.shooter_17mm_1_barrel_heat;
  gimbal_rfr_data.shooter_cooling = rpp_data.shooter_barrel_cooling_value;
  gimbal_rfr_data.shooter_heat_limit = rpp_data.shooter_barrel_heat_limit;

  // Hurt
  robot_rfr_data_.hurt_module_id = rhp_data.module_id;
  robot_rfr_data_.hurt_reason = rhp_data.hp_deduction_reason;
};

void Robot::updatePwrState() {
  PwrState pre_state = pwr_state_;
  PwrState next_state = pre_state;
  if (pre_state == PwrState::kDead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    next_state = PwrState::kResurrection;
  } else if (pre_state == PwrState::kResurrection) {
    if (is_imu_caled_offset_) {
      next_state = PwrState::kWorking;
    }
  } else if (pre_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

#pragma endregion

#pragma region 获取控制指令
void Robot::genModulesCmd() {
  bool rev_gimbal_flag = false;
  static uint8_t last_rev_chassis_cnt = 0;
  static uint8_t last_rev_gimbal_cnt = 0;
  GimbalChassisComm::ChassisData::GimbalPart &chassis_data =
      gc_comm_ptr_->chassis_data().gp;

  if (last_rev_gimbal_cnt != chassis_data.rev_gimbal_cnt) {
    last_rev_gimbal_tick_ = work_tick_;
    rev_gimbal_flag = true;
    // rev_gimbal_flag = false; // TODO:调试
    chassis_ptr_->revChassis();
  }
  last_rev_gimbal_cnt = chassis_data.rev_gimbal_cnt;
  if (last_rev_chassis_cnt != chassis_data.rev_chassis_cnt) {
    last_rev_chassis_tick_ = work_tick_;
    chassis_ptr_->revChassis();
  }
  last_rev_chassis_cnt = chassis_data.rev_chassis_cnt;

  ChassisCmd chassis_cmd = {chassis_data.v_x_raw, chassis_data.v_y_raw, 0.0f};
  chassis_ptr_->setWorkingMode(chassis_data.chassis_working_mode);
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setUseCapFlag(chassis_data.use_cap_flag);
  chassis_ptr_->setGyroDir(chassis_data.gyro_dir);
  chassis_ptr_->setGyroMode(chassis_data.gyro_mode);
  chassis_ptr_->setRevGimbalFlag(rev_gimbal_flag);
}

#pragma endregion

#pragma region 执行任务
void Robot::runOnDead() {
  resetDataOnDead();
  standby();
};

void Robot::runOnResurrection() {
  resetDataOnResurrection();
  standby();
};

void Robot::runOnWorking() {
  HW_ASSERT(buzzer_ptr_ != nullptr, "buzzer pointer is null", buzzer_ptr_);
  if (buzzer_ptr_->is_playing()) {
    buzzer_ptr_->play();
  }
  genModulesCmd();
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->run();
};

void Robot::runAlways() {
  setCommData();
  sendCommData();
};

void Robot::standby() {
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();
};

#pragma endregion

#pragma region 数据重置函数
void Robot::reset() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源
};
void Robot::resetDataOnDead() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源
};
void Robot::resetDataOnResurrection() {};

#pragma endregion

#pragma region 通信数据设置函数
void Robot::setCommData() {
  // setGimbalChassisCommData();
  setUiDrawerData();
};

// void Robot::setGimbalChassisCommData() {
//   HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
//             gc_comm_ptr_);
//   HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);
//   HW_ASSERT(shooter_ptr_ != nullptr, "Shooter pointer is null",
//   shooter_ptr_);

//   // gimbal
//   // shooter
// };

void Robot::setUiDrawerData() {
  // Chassis
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  ui_drawer_.setIsWheelMotorsOnline(chassis_ptr_->getIsAllWheelOnline());
  // ui_drawer_.setIsWheelMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setIsSteerMotorsOnline(chassis_ptr_->getIsAllSteerOnline());
  // ui_drawer_.setIsSteerMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setChassisThetaI2r(chassis_ptr_->getThetaI2r());
  ui_drawer_.setChassisWorkState(chassis_ptr_->getPwrState());
  ui_drawer_.setChassisWorkingMode(chassis_ptr_->getWorkingMode());
  ui_drawer_.setChassisManualCtrlSrc(
      gc_comm_ptr_->main_board_data().gp.manual_ctrl_src);

  // Cap
  HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  ui_drawer_.setCapPwrPercent(cap_ptr_->getRemainingPower());
  // ui_drawer_.setCapPwrPercent(65.7); // TODO:UI测试

  // Referee
  ui_drawer_.setSenderId(robot_id_);

  ui_drawer_.setHeat(gc_comm_ptr_->referee_data().cp.shooter_heat);
  ui_drawer_.setHeatLimit(gc_comm_ptr_->referee_data().cp.shooter_heat_limit);

  if (robot_rfr_data_.hurt_reason ==
          static_cast<uint8_t>(HurtReason::kArmorHit) ||
      robot_rfr_data_.hurt_reason ==
          static_cast<uint8_t>(HurtReason::kArmorCollision)) {
    ui_drawer_.setIsArmorHit(true);
    ui_drawer_.setArmorIdHit(robot_rfr_data_.hurt_module_id);
  } else {
    ui_drawer_.setIsArmorHit(false);
    ui_drawer_.setArmorIdHit(0);
  }

  // Gimbal
  ui_drawer_.setIsGimbalMotorsOnline(
      gc_comm_ptr_->gimbal_data().gp.is_gimbal_motors_online);
  // ui_drawer_.setIsGimbalMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setGimbalJointAngPitchFdb(
      gc_comm_ptr_->gimbal_data().gp.pitch_fdb);
  ui_drawer_.setGimbalWorkState(
      gc_comm_ptr_->gimbal_data().gp.gimbal_pwr_state);
  // ui_drawer_.setGimbalWorkState(PwrState::kResurrection); //TODO:UI测试
  ui_drawer_.setGimbalWorkingMode(
      gc_comm_ptr_->gimbal_data().gp.gimbal_working_mode);
  ui_drawer_.setGimbalCtrlMode(gc_comm_ptr_->gimbal_data().gp.gimbal_ctrl_mode);
  ui_drawer_.setGimbalManualCtrlSrc(
      gc_comm_ptr_->main_board_data().gp.manual_ctrl_src);

  // Shooter
  ui_drawer_.setIsShooterMotorsOnline(
      gc_comm_ptr_->shooter_data().gp.is_shooter_motors_online);
  // ui_drawer_.setIsShooterMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setShooterStuckFlag(
      gc_comm_ptr_->shooter_data().gp.is_shooter_stuck);
  // ui_drawer_.setShooterStuckFlag(true); //TODO:UI测试
  ui_drawer_.setFeedStuckStatus(
      gc_comm_ptr_->shooter_data().gp.feed_stuck_state);
  ui_drawer_.setShooterWorkState(
      gc_comm_ptr_->shooter_data().gp.shooter_pwr_state);
  ui_drawer_.setShooterCtrlMode(
      gc_comm_ptr_->shooter_data().gp.shooter_ctrl_mode);
  ui_drawer_.setShooterManualCtrlSrc(
      gc_comm_ptr_->main_board_data().gp.manual_ctrl_src);

  // Vision
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  bool is_vision_valid = gc_comm_ptr_->vision_data().gp.is_enemy_detected;
  ui_drawer_.setIsVisionOnline(gc_comm_ptr_->vision_data().gp.is_vision_online);
  // ui_drawer_.setIsVisionOnline(false); //TODO:UI测试
  ui_drawer_.setIsVisionValid(is_vision_valid);
  ui_drawer_.setVisTgtX(gc_comm_ptr_->vision_data().gp.vtm_x, is_vision_valid);
  ui_drawer_.setVisTgtY(gc_comm_ptr_->vision_data().gp.vtm_y, is_vision_valid);

  static uint8_t last_ui_refresh_cnt = 0;
  if (gc_comm_ptr_->referee_data().gp.ui_refresh_cnt != last_ui_refresh_cnt) {
    ui_drawer_.refresh();
  }
  last_ui_refresh_cnt = gc_comm_ptr_->referee_data().gp.ui_refresh_cnt;
};

#pragma endregion

#pragma region 通信数据发送函数

void Robot::sendCommData() {
  sendCanData();
  sendUsartData();
};
void Robot::sendCanData() {
  sendSteersMotorData();
  if (work_tick_ % 10 == 0) {
    sendCapData();
  }

  // CAN2负载高，交替发送降低负载
  if (work_tick_ % 2 == 0) {
    sendWheelsMotorData();
  } else if (work_tick_ % 2 == 1) {
    sendGimbalChassisCommData();
  }
};
void Robot::sendWheelsMotorData() {
  Motor *dev_ptr = nullptr;
  for (size_t i = 0; i < kWheelMotorNum; i++) {
    dev_ptr = motor_wheels_ptr_[i];

    HW_ASSERT(dev_ptr != nullptr, "Wheel Motor pointer %d is null", midx);
    dev_ptr->setNeedToTransmit();
  }
};
void Robot::sendSteersMotorData() {
  Motor *dev_ptr = nullptr;
  for (size_t i = 0; i < kSteerMotorNum; i++) {
    dev_ptr = motor_steers_ptr_[i];

    HW_ASSERT(dev_ptr != nullptr, "Steer Motor pointer %d is null", midx);
    dev_ptr->setNeedToTransmit();
  }
};

void Robot::sendCapData() {
  HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  cap_ptr_->setNeedToTransmit();
};
void Robot::sendGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  gc_comm_ptr_->setNeedToTransmit();
};
void Robot::sendUsartData() {
  if (work_tick_ % 5 == 0) {
    sendRefereeData();
  }
};
uint32_t debug_tick[3] = {0};
void Robot::sendRefereeData() {
  debug_tick[0]++;
  if (ui_drawer_.encode(rfr_tx_data_, rfr_tx_data_len_)) {
    HAL_UART_Transmit_DMA(&huart6, rfr_tx_data_, rfr_tx_data_len_);
    debug_tick[1]++;
  }
};
#pragma endregion

#pragma region 注册函数

void Robot::registerChassis(Chassis *ptr) {
  HW_ASSERT(ptr != nullptr, "Chassis pointer is null", ptr);
  chassis_ptr_ = ptr;
};
void Robot::registerBuzzer(Buzzer *ptr) {
  HW_ASSERT(ptr != nullptr, "Buzzer pointer is null", ptr);
  buzzer_ptr_ = ptr;
};
void Robot::registerImu(Imu *ptr) {
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};

void Robot::registerMotorWheels(Motor *dev_ptr, uint8_t idx) {
  HW_ASSERT(dev_ptr != nullptr, "Wheel Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum,
            "Wheel Motor index is out of range", idx);
  if (idx >= kWheelMotorNum || motor_wheels_ptr_[idx] == dev_ptr) {
    return;
  }
  motor_wheels_ptr_[idx] = dev_ptr;
};

void Robot::registerMotorSteers(Motor *dev_ptr, uint8_t idx) {
  HW_ASSERT(dev_ptr != nullptr, "Steer Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < kSteerMotorNum,
            "Steer Motor index is out of range", idx);
  if (idx >= kSteerMotorNum || motor_steers_ptr_[idx] == dev_ptr) {
    return;
  }
  motor_steers_ptr_[idx] = dev_ptr;
}

void Robot::registerCap(Cap *dev_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Cap pointer is null", dev_ptr);
  if (cap_ptr_ == dev_ptr) {
    return;
  }

  cap_ptr_ = dev_ptr;
};

void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
  if (gc_comm_ptr_ == dev_ptr) {
    return;
  }

  gc_comm_ptr_ = dev_ptr;
};

void Robot::registerReferee(Referee *dev_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Referee pointer is null", dev_ptr);
  if (referee_ptr_ == dev_ptr) {
    return;
  }

  referee_ptr_ = dev_ptr;
};

void Robot::registerPerformancePkg(PerformancePkg *ptr) {
  HW_ASSERT(ptr != nullptr, "PerformancePkg pointer is null", ptr);
  if (rfr_performance_pkg_ptr_ == ptr) {
    return;
  }
  rfr_performance_pkg_ptr_ = ptr;
};
void Robot::registerPowerHeatPkg(PowerHeatPkg *ptr) {
  HW_ASSERT(ptr != nullptr, "PowerHeatPkg pointer is null", ptr);
  if (rfr_power_heat_pkg_ptr_ == ptr) {
    return;
  }
  rfr_power_heat_pkg_ptr_ = ptr;
};
void Robot::registerShooterPkg(ShooterPkg *ptr) {
  HW_ASSERT(ptr != nullptr, "ShooterPkg pointer is null", ptr);
  if (rfr_shooter_pkg_ptr_ == ptr) {
    return;
  }
  rfr_shooter_pkg_ptr_ = ptr;
};
void Robot::registerHurtPkg(HurtPkg *ptr) {
  HW_ASSERT(ptr != nullptr, "HurtPkg pointer is null", ptr);
  if (rfr_hurt_pkg_ptr_ == ptr) {
    return;
  }
  rfr_hurt_pkg_ptr_ = ptr;
};
#pragma endregion
/* Private function definitions
 * ----------------------------------------------*/
} // namespace robot