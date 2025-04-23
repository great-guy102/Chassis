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
    .bullet_speed = 15.5f,    ///< 弹丸初速度，单位：m/s
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
  updateRcData();
};

void Robot::updateImuData() {
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isOffsetCalcFinished()) {
    is_imu_caled_offset_ = true;
  }
};

void Robot::updateRcData() {
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);
  if (manual_ctrl_src_ == ManualCtrlSrc::kRc) {
    if (rc_ptr_->isUsingKeyboardMouse()) {
      setManualCtrlSrc(ManualCtrlSrc::kKb);
    }
  } else if (manual_ctrl_src_ == ManualCtrlSrc::kKb) {
    if (rc_ptr_->isRcSwitchChanged()) { // 不检测摇杆变化，防止控制源来回切换
      setManualCtrlSrc(ManualCtrlSrc::kRc);
    }
  }
};

// float inf1 = 0.0f, inf2 = 0.0f; //TODO：测试
// Robot::HurtPkg::Data rhr_data = kDefaultRobotHurtData; //TODO:测试
void Robot::updateRfrData() {
  HW_ASSERT(referee_ptr_ != nullptr, "RFR pointer is null", referee_ptr_);
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);

  PerformancePkg::Data rpp_data = kDefaultRobotPerformanceData;
  PowerHeatPkg::Data rph_data = kDefaultRobotPowerHeatData;
  ShooterPkg::Data rsp_data = kDefaultRobotShooterData;
  HurtPkg::Data rhr_data = kDefaultRobotHurtData;
  static uint8_t rfr_bullet_shot_cnt = 0;

  if (!referee_ptr_->isOffline()) {
    rpp_data = rfr_performance_pkg_ptr_->getData();
    rph_data = rfr_power_heat_pkg_ptr_->getData();
    rsp_data = rfr_shooter_pkg_ptr_->getData();
    rhr_data = rfr_hurt_pkg_ptr_->getData();

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
    robot_id_ = Robot::RobotId::kBlueStandard3;
  }
  ui_drawer_.setSenderId(robot_id_);

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
  robot_rfr_data_.hurt_module_id = rhr_data.module_id;
  robot_rfr_data_.hurt_reason = rhr_data.hp_deduction_reason;
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

#pragma region 生成控制指令
void Robot::genModulesCmd() {
  // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  if (manual_ctrl_src_ == ManualCtrlSrc::kRc) {
    genModulesCmdFromRc();
  } else if (manual_ctrl_src_ == ManualCtrlSrc::kKb) {
    genModulesCmdFromKb();
  }
};

void Robot::genModulesCmdFromRc() {
  // 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  RcSwitchState l_switch = rc_ptr_->rc_l_switch();
  RcSwitchState r_switch = rc_ptr_->rc_r_switch();
  float rc_wheel = rc_ptr_->rc_wheel();

  Chassis::WorkingMode chassis_working_mode = Chassis::WorkingMode::Depart;
  Gimbal::WorkingMode gimbal_working_mode = Gimbal::WorkingMode::Normal;
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::kShoot;
  Chassis::GyroDir gyro_dir = Chassis::GyroDir::Unspecified;
  Chassis::GyroMode gyro_mode = Chassis::GyroMode::ConstW;

  CtrlMode gimbal_ctrl_mode = CtrlMode::kManual;
  CtrlMode shooter_ctrl_mode = CtrlMode::kManual;

  bool use_cap_flag = (rc_wheel > 0.9f);
  // bool shoot_flag = (rc_wheel > 0.9f); // 自动模式也能手动发弹;
  bool shoot_flag = false;      // TODO：调试
  bool rev_gimbal_flag = false; // TODO:掉头模式，只建议分离/跟随模式使用
  bool rev_chassis_flag = false;

  static uint8_t rev_gimbal_rc_cnt = 0; // 板间通信防丢包机制

  // TODO: 后续需要加入慢拨模式
  if (l_switch == RcSwitchState::kUp) {
    // * 左上
    chassis_working_mode = Chassis::WorkingMode::Depart;

    if (r_switch == RcSwitchState::kUp) {
      // * 左上右上
      gimbal_working_mode = Gimbal::WorkingMode::Sentry;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左上右中
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kManual;
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左上右下
      // TODO：云台PID测试模式
      // gimbal_working_mode = Gimbal::WorkingMode::PidTest;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    }
  } else if (l_switch == RcSwitchState::kMid) {
    // * 左中
    chassis_working_mode = Chassis::WorkingMode::Follow;

    if (r_switch == RcSwitchState::kUp) {
      // * 左中右上
      gimbal_ctrl_mode = CtrlMode::kManual;
      shooter_ctrl_mode = CtrlMode::kManual;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左中右中
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kManual;
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左中右下
      // TODO：云台PID测试模式
      // gimbal_working_mode = Gimbal::WorkingMode::PidTest;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    }
  } else if (l_switch == RcSwitchState::kDown) {
    // * 左下
    chassis_working_mode = Chassis::WorkingMode::Gyro;
    gyro_dir = Chassis::GyroDir::Clockwise;
    gyro_mode = Chassis::GyroMode::SinW;
    if (r_switch == RcSwitchState::kUp) {
      // * 左下右上
      gimbal_working_mode = Gimbal::WorkingMode::Sentry;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左下右中
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kManual;
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左下右下
      // TODO：云台PID测试模式
      // gimbal_working_mode = Gimbal::WorkingMode::PidTest;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    }
  }

  ChassisState chassis_cmd = {0};
  chassis_cmd.v_x = hello_world::Bound(rc_ptr_->rc_rv(), -1, 1);
  chassis_cmd.v_y = hello_world::Bound(-rc_ptr_->rc_rh(), -1, 1);
  chassis_cmd.w = 0.0f;
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setGyroDir(gyro_dir);
  chassis_ptr_->setGyroMode(gyro_mode);
  chassis_ptr_->setUseCapFlag(use_cap_flag);

  // TODO：跟随模式云台前馈记录
  float omega_feedforward = hello_world::Bound(-rc_ptr_->rc_lh(), -1, 1);
  chassis_ptr_->setOmegaFeedforward(omega_feedforward);

  if ((work_tick_ - last_rev_gimbal_tick_ > 200) &&
      (work_tick_ - last_rev_chassis_tick_ > 200)) {
    chassis_ptr_->setRevGimbalFlag(rev_gimbal_flag);
    if (rev_gimbal_flag) {
      rev_gimbal_rc_cnt = (rev_gimbal_rc_cnt == 0 ? 1 : 0);
      chassis_ptr_->revChassis();
      last_rev_gimbal_tick_ = work_tick_;
    } else {
      if (rev_chassis_flag) {
        chassis_ptr_->revChassis();
        last_rev_chassis_tick_ = work_tick_;
      }
    }
  }

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(rc_ptr_->rc_lv(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-rc_ptr_->rc_lh(), -1,
                                      1); // 右手系，z轴竖直向上，左转为正
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);
  gimbal_ptr_->setRevGimbalCnt(rev_gimbal_rc_cnt);

  shooter_ptr_->setCtrlMode(shooter_ctrl_mode);
  shooter_ptr_->setWorkingMode(shooter_working_mode);
  shooter_ptr_->setShootFlag(shoot_flag);
};

void Robot::genModulesCmdFromKb() {
  // // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // // 指令应该通过各个模块的接口发送给各个模块
  Chassis::WorkingMode chassis_working_mode = chassis_ptr_->getWorkingMode();
  Gimbal::WorkingMode gimbal_working_mode = gimbal_ptr_->getWorkingMode();
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::kShoot;
  Chassis::GyroDir gyro_dir = Chassis::GyroDir::Unspecified;
  Chassis::GyroMode gyro_mode = Chassis::GyroMode::ConstW;

  CtrlMode gimbal_ctrl_mode = CtrlMode::kManual;
  CtrlMode shooter_ctrl_mode = CtrlMode::kManual;

  bool use_cap_flag = false;
  bool shoot_flag = false;      // 自动模式也能手动发弹;
  bool rev_gimbal_flag = false; // TODO:掉头模式，只建议分离/跟随模式使用
  bool rev_chassis_flag = false;

  static uint8_t rev_gimbal_kb_cnt = 0; // 板间通信防丢包机制

  if (rc_ptr_->key_Q()) {
    chassis_working_mode = Chassis::WorkingMode::Gyro;
    gyro_dir = Chassis::GyroDir::Clockwise;
    gyro_mode = Chassis::GyroMode::SinW;
  } else if (rc_ptr_->key_E()) {
    chassis_working_mode = Chassis::WorkingMode::Follow;
  } else {
    if (chassis_working_mode == Chassis::WorkingMode::Depart) {
      chassis_working_mode = Chassis::WorkingMode::Follow;
    }
  }
  if (rc_ptr_->key_R()) {
    ui_drawer_.refresh();
  }

  if (rc_ptr_->key_F()) {
  }

  if (rc_ptr_->key_SHIFT()) {
    use_cap_flag = true;
  }
  if (rc_ptr_->key_Z()) {
    shooter_working_mode = Shooter::WorkingMode::kBackward;
  }
  if (rc_ptr_->key_X()) {
    shooter_ctrl_mode = CtrlMode::kAuto;
  }
  if (rc_ptr_->key_C()) {
    rev_gimbal_flag = true;
  }

  if (rc_ptr_->mouse_l_btn()) {
    shoot_flag = true;
  }
  if (rc_ptr_->mouse_r_btn()) {
    gimbal_ctrl_mode = CtrlMode::kAuto;
  }

  ChassisState chassis_cmd = {0};
  chassis_cmd.v_x = hello_world::Bound(rc_ptr_->key_W() - rc_ptr_->key_S(), -1,
                                       1); // 前进后退
  chassis_cmd.v_y = hello_world::Bound(rc_ptr_->key_A() - rc_ptr_->key_D(), -1,
                                       1); // 左右平移
  chassis_cmd.w = 0.0f;
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setGyroDir(gyro_dir);
  chassis_ptr_->setGyroMode(gyro_mode);
  chassis_ptr_->setUseCapFlag(use_cap_flag);

  if ((work_tick_ - last_rev_gimbal_tick_ > 200) &&
      (work_tick_ - last_rev_chassis_tick_ > 200)) {
    chassis_ptr_->setRevGimbalFlag(rev_gimbal_flag);
    if (rev_gimbal_flag) {
      rev_gimbal_kb_cnt = (rev_gimbal_kb_cnt == 0 ? 1 : 0);
      chassis_ptr_->revChassis();
      last_rev_gimbal_tick_ = work_tick_;
    } else {
      if (rev_chassis_flag) {
        chassis_ptr_->revChassis();
        last_rev_chassis_tick_ = work_tick_;
      }
    }
  }

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(-0.01 * rc_ptr_->mouse_y(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-0.01 * rc_ptr_->mouse_x(), -1, 1);
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);
  gimbal_ptr_->setRevGimbalCnt(rev_gimbal_kb_cnt); // TODO:板间通信防丢包机制

  shooter_ptr_->setCtrlMode(shooter_ctrl_mode);
  shooter_ptr_->setWorkingMode(shooter_working_mode);
  shooter_ptr_->setShootFlag(shoot_flag);
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
  setGimbalChassisCommData();
  setUiDrawerData();
};

void Robot::setGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);
  HW_ASSERT(shooter_ptr_ != nullptr, "Shooter pointer is null", shooter_ptr_);

  // gimbal
  GimbalChassisComm::GimbalData::ChassisPart &gimbal_data =
      gc_comm_ptr_->gimbal_data().cp;
  gimbal_data.rev_gimbal_cnt = gimbal_ptr_->getRevGimbalCnt();
  gimbal_data.yaw_delta = gimbal_ptr_->getNormCmdDelta().yaw;
  gimbal_data.pitch_delta = gimbal_ptr_->getNormCmdDelta().pitch;
  gimbal_data.ctrl_mode = gimbal_ptr_->getCtrlMode();
  gimbal_data.working_mode = gimbal_ptr_->getWorkingMode();

  // shooter
  GimbalChassisComm::ShooterData::ChassisPart &shooter_data =
      gc_comm_ptr_->shooter_data().cp;
  shooter_data.setShootFlag(shooter_ptr_->getShootFlag());
  shooter_data.ctrl_mode = shooter_ptr_->getCtrlMode();
  shooter_data.working_mode = shooter_ptr_->getWorkingMode();
};

void Robot::setUiDrawerData() {
  // ui_drawer_.setSenderId(RobotId::kRedStandard3);
  // Chassis
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  ui_drawer_.setChassisWorkState(chassis_ptr_->getPwrState());
  ui_drawer_.setIsWheelMotorsOnline(chassis_ptr_->getIsAllWheelOnline());
  // ui_drawer_.setIsWheelMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setIsSteerMotorsOnline(chassis_ptr_->getIsAllSteerOnline());
  // ui_drawer_.setIsSteerMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setChassisWorkingMode(chassis_ptr_->getWorkingMode());
  ui_drawer_.setChassisManualCtrlSrc(manual_ctrl_src_);
  ui_drawer_.setChassisThetaI2r(chassis_ptr_->getThetaI2r());

  // Gimbal
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
  ui_drawer_.setGimbalWorkState(gc_comm_ptr_->gimbal_data().gp.pwr_state);
  // ui_drawer_.setGimbalWorkState(PwrState::kResurrection); //TODO:UI测试
  ui_drawer_.setIsGimbalMotorsOnline(
      gc_comm_ptr_->gimbal_data().gp.is_gimbal_motors_online);
  // ui_drawer_.setIsGimbalMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setGimbalCtrlMode(gimbal_ptr_->getCtrlMode());
  ui_drawer_.setGimbalManualCtrlSrc(manual_ctrl_src_);
  ui_drawer_.setGimbalWorkingMode(gimbal_ptr_->getWorkingMode());
  ui_drawer_.setGimbalJointAngPitchFdb(
      gc_comm_ptr_->gimbal_data().gp.pitch_fdb);

  // Shooter
  HW_ASSERT(shooter_ptr_ != nullptr, "Shooter FSM pointer is null",
            shooter_ptr_);
  ui_drawer_.setShooterWorkState(gc_comm_ptr_->shooter_data().gp.pwr_state);
  ui_drawer_.setIsShooterMotorsOnline(
      gc_comm_ptr_->shooter_data().gp.is_shooter_motors_online);
  // ui_drawer_.setIsShooterMotorsOnline(false); // TODO:UI测试
  ui_drawer_.setShooterCtrlMode(shooter_ptr_->getCtrlMode());
  ui_drawer_.setShooterManualCtrlSrc(manual_ctrl_src_);
  ui_drawer_.setShooterStuckFlag(
      gc_comm_ptr_->shooter_data().gp.is_shooter_stuck);
  // ui_drawer_.setShooterStuckFlag(true); //TODO:UI测试
  ui_drawer_.setFeedStuckStatus(
      gc_comm_ptr_->shooter_data().gp.feed_stuck_state);
  ui_drawer_.setHeat(gc_comm_ptr_->referee_data().cp.shooter_heat);
  ui_drawer_.setHeatLimit(gc_comm_ptr_->referee_data().cp.shooter_heat_limit);

  // Cap
  HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  ui_drawer_.setCapPwrPercent(cap_ptr_->getRemainingPower());
  // ui_drawer_.setCapPwrPercent(65.7); // TODO:UI测试

  // vision
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  bool is_vision_valid = gc_comm_ptr_->vision_data().gp.is_enemy_detected;
  ui_drawer_.setIsVisionOnline(gc_comm_ptr_->vision_data().gp.is_vision_online);
  // ui_drawer_.setIsVisionOnline(false); //TODO:UI测试
  ui_drawer_.setIsVisionValid(is_vision_valid);
  ui_drawer_.setVisTgtX(gc_comm_ptr_->vision_data().gp.vtm_x, is_vision_valid);
  ui_drawer_.setVisTgtY(gc_comm_ptr_->vision_data().gp.vtm_y, is_vision_valid);

  // hurt
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

  // referee_ptr_->setTxPkg(ui_drawer_.))
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
  if (work_tick_ > 1000 && work_tick_ % 2 == 1) {
    sendGimbalChassisCommData();
  }
  if (work_tick_ % 2 == 0) {
    sendWheelsMotorData();
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
void Robot::registerGimbal(Gimbal *ptr) {
  HW_ASSERT(ptr != nullptr, "Gimbal pointer is null", ptr);
  gimbal_ptr_ = ptr;
};
void Robot::registerShooter(Shooter *ptr) {
  HW_ASSERT(ptr != nullptr, "Shooter pointer is null", ptr);
  shooter_ptr_ = ptr;
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

void Robot::registerRc(DT7 *ptr) {
  HW_ASSERT(ptr != nullptr, "DT7 pointer is null", ptr);
  if (rc_ptr_ == ptr) {
    return;
  }
  rc_ptr_ = ptr;
}

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