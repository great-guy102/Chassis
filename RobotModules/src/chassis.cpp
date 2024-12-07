/** 
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/
// DEBUG: 
float wheel_speed_fdb_debug = 0;
float wheel_speed_ref_debug = 0;
// float steer_speed_fdb_debug = 0;
// float steer_speed_ref_debug = 0;

// TODO: norm_cmd_和cmd_好像没有关联起来，非常奇怪
// Robot的runOnWorking里产生各个模块的norm_cmd_
namespace robot
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
void Chassis::update()
{
  updateData();
  updatePwrState();
};

void Chassis::updateData()
{
  updateWorkTick();
  updateGimbalBoard();
  updateMotor();
  updateCap();
  updateIsPowerOn();
};

/**
 * 更新电源状态
 * 
 * 无论处于任何状态，只要掉电就切换到死亡状态。
 * 死亡状态下，如果上电，则切到复活状态；
 * 复活状态下，如果有轮或舵电机上电完毕（底盘已经准备完毕），且云台板 IMU 准备就绪，则切到工作状态；
 * 工作状态下，保持当前状态；
 * 其他状态，认为是死亡状态。
 */
void Chassis::updatePwrState()
{
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_power_on_) {
    setPwrState(PwrState::Dead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::Dead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_power_on_) {
      next_state = PwrState::Resurrection;
    }
  } else if (current_state == PwrState::Resurrection) {
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，底盘就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么只要有轮或舵电机上电完毕，就认为底盘就准备好了？
    // 因为底盘逆解不需要所有轮或舵电机都在线也可以运行
    // 在后续正常工作状态下，会对轮或舵电机的状态进行检测
    // 如果有轮或舵电机掉电，会进行专门的处理，比如 pid 清空、can 发无效数据等
    // 3. 所有控制模式都需要 yaw 轴电机的角度，为什么不判断 yaw 轴电机的状态？
    // 因为 yaw 轴电机的状态不影响底盘的运动解算
    // 当 yaw 轴电机离线时，底盘会按照底盘坐标系进行解算（yaw 电机离线后数据清空，默认返回 0，能跑但是疯了，但总比不能跑强）
    // 当 yaw 轴电机上电时，底盘会按照图传坐标系进行解算
    if (is_gimbal_imu_ready_ && (is_any_wheel_online_ || is_any_steer_online_)) {
      next_state = PwrState::Working;
    }
  } else if (current_state == PwrState::Working) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::Dead;
  }
  setPwrState(next_state);
};

void Chassis::updateGimbalBoard()
{
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板 IMU 准备就绪，使得可以在无云台板下工作
  HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gc_comm_ptr_);
  if (gc_comm_ptr_->isOffline()) {
    is_gimbal_imu_ready_ = true;
  } else {
    is_gimbal_imu_ready_ = gc_comm_ptr_->main_board_data().gp.is_gimbal_imu_ready;
  }
};

void Chassis::updateMotor()
{
  Motor *wheel_motor_ptr = nullptr;
  Motor *steer_motor_ptr = nullptr;
  WheelMotorIdx wmis[4] = {
    kWheelMotorIdxLeftFront,
    kWheelMotorIdxLeftRear,
    kWheelMotorIdxRightRear,
    kWheelMotorIdxRightFront,
  };
  SteerMotorIdx smis[4] = {
    kSteerMotorIdxLeftFront,
    kSteerMotorIdxLeftRear,
    kSteerMotorIdxRightRear,
    kSteerMotorIdxRightFront,
  };
  bool is_all_wheel_online = true;
  bool is_any_wheel_online = false;
  bool is_all_steer_online = true;
  bool is_any_steer_online = false;

  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    SteerMotorIdx smi = smis[i];
    wheel_motor_ptr = wheel_motor_ptr_[wmi];
    steer_motor_ptr = steer_motor_ptr_[smi];

    HW_ASSERT(wheel_motor_ptr != nullptr, "pointer to wheel motor %d is nullptr", wmi);
    HW_ASSERT(steer_motor_ptr != nullptr, "pointer to steer motor %d is nullptr", smi);
    if (wheel_motor_ptr->isOffline()) {
      wheel_speed_fdb_[wmi] = 0.0;
      wheel_current_fdb_[wmi] = 0.0;
      is_all_wheel_online = false;
    } else {
      is_any_wheel_online = true;
      wheel_speed_fdb_[wmi] = wheel_motor_ptr->vel();
      wheel_current_fdb_[wmi] = wheel_motor_ptr->curr();
    }
    
    if (steer_motor_ptr->isOffline()) {
      steer_speed_fdb_[smi] = 0.0;
      steer_current_fdb_[smi] = 0.0;
      is_all_steer_online = false;
    } else {
      is_any_steer_online = true;
      steer_speed_fdb_[smi] = steer_motor_ptr->vel();
      steer_angle_fdb_[smi] = steer_motor_ptr->angle();
      steer_current_fdb_[smi] = steer_motor_ptr->curr();
    }
  }
  is_all_wheel_online_ = is_all_wheel_online;
  is_any_wheel_online_ = is_any_wheel_online;
  is_all_steer_online_ = is_all_steer_online;
  is_any_steer_online_ = is_any_steer_online;

  HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to Yaw motor is nullptr", yaw_motor_ptr_);
  if (yaw_motor_ptr_->isOffline()) {
    theta_i2r_ = 0.0;
  } else {
    theta_i2r_ = -yaw_motor_ptr_->angle();
  }
};

void Chassis::updateCap()
{
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (cap_ptr_->isOffline()) {
    is_high_spd_enabled_ = false;
    cap_remaining_energy_ = 0.0f;
  } else {
    is_high_spd_enabled_ = cap_ptr_->isUsingSuperCap();
    cap_remaining_energy_ = cap_ptr_->getRemainingPower();
  }
};

void Chassis::updateIsPowerOn()
{
  is_power_on_ = (is_any_wheel_online_ && is_all_steer_online_) || rfr_data_.is_pwr_on;
  if (!is_power_on_) {
    last_pwr_off_tick_ = work_tick_;
  }
};

#pragma endregion

#pragma region 执行任务
void Chassis::run()
{
  if (pwr_state_ == PwrState::Dead) {
    runOnDead();
  } else if (pwr_state_ == PwrState::Resurrection) {
    runOnResurrection();
  } else if (pwr_state_ == PwrState::Working) {
    runOnWorking();
  } else {
    runOnDead();
  }
};

void Chassis::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Chassis::runOnResurrection()
{
  resetDataOnResurrection();
  setCommData(false);
};

void Chassis::runOnWorking()
{
  revNormCmd();
  calcMotorsRef();
  // calcWheelLimitedSpeedRef();
  calcWheelCurrentRef();
  calcSteerVoltageRef();
  setCommData(true);
};

void Chassis::standby()
{
  resetDataOnStandby();
  setCommData(false);
};
#pragma endregion

#pragma region 工作状态下，获取控制指令的函数

void Chassis::revNormCmd()
{
  float beta = 0.5;
  Cmd cmd = norm_cmd_;

  if (working_mode_ == WorkingMode::Depart) {
    // 分离模式不对 norm_cmd_ 进行额外处理
  } else if (working_mode_ == WorkingMode::Gyro) {
    // 陀螺模式下，如果外部没有设置陀螺旋转方向，
    // 则每次进入陀螺模式时，随机选择一个方向
    if (gyro_dir_ == GyroDir::NotRotate) {
      if (rand() % 2 == 0) {
        gyro_dir_ = GyroDir::Clockwise;
      } else {
        gyro_dir_ = GyroDir::AntiClockwise;
      }
    }
    // 小陀螺模式下，旋转分量为定值
    cmd.w = config_.gyro_rot_spd * (int8_t)gyro_dir_;

  } else if (working_mode_ == WorkingMode::Follow) {
    // 跟随模式下，更新跟随目标
    float theta_ref = 0.0f;
    float theta_fdb = theta_i2r_;
    float omega_feedforward = config_.yaw_sensitivity * omega_feedforward_;
    if (rev_head_flag_) {
      theta_ref = PI;
    }
    follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, &omega_feedforward, &cmd.w);
  } 

  if (working_mode_ != WorkingMode::Gyro) {
    gyro_dir_ = GyroDir::NotRotate;
  }

  if (is_high_spd_enabled_) {
    cmd *= 1.5;
    beta = 1.0;
  }

  //TODO：修改限幅逻辑，保证限幅后平移速度方向不变
  cmd.v_x = hello_world::Bound(cmd.v_x * config_.normal_trans_vel, -config_.max_trans_vel, config_.max_trans_vel);
  cmd.v_y = hello_world::Bound(cmd.v_y * config_.normal_trans_vel, -config_.max_trans_vel, config_.max_trans_vel);
  cmd.w = hello_world::Bound(cmd.w, -config_.max_rot_spd, config_.max_rot_spd);

  setCmdSmoothly(cmd, beta);
};

void Chassis::calcMotorsRef()
{
  // 底盘坐标系下，x轴正方向为底盘正前方，y轴正方向为底盘正左方，z轴正方向为底盘正上方
  // 轮子顺序按照象限顺序进行编号：左前，左后，右后，右前
  HW_ASSERT(ik_solver_ptr_ != nullptr, "pointer to IK solver is nullptr", ik_solver_ptr_);
  hello_world::chassis_ik_solver::MoveVec move_vec(cmd_.v_x, cmd_.v_y, cmd_.w);
  float theta_i2r = theta_i2r_;
  ik_solver_ptr_->solve(move_vec, theta_i2r, steer_angle_fdb_);
  ik_solver_ptr_->getRotSpdAll(wheel_speed_ref_);
  ik_solver_ptr_->getThetaVelRefAll(steer_angle_ref_);
};

void Chassis::calcWheelLimitedSpeedRef()
{
  hello_world::power_limiter::PwrLimitRuntimeParams runtime_params = {
      .is_referee_online = rfr_data_.is_rfr_on,  // 裁判系统是否在线
      .p_rfr_max = rfr_data_.pwr_limit,          // 裁判系统给出功率上限
      .z_rfr_measure = rfr_data_.pwr_buffer,     // 裁判系统给出剩余缓冲能量值
      .p_rfr_measure = rfr_data_.pwr,            // 裁判系统给出实际功率
      .p_dummy_max = rfr_data_.pwr_limit,        // TODO: 这个参数和p_rfr_max有什么区别？
  };
  if (!cap_ptr_->isOffline()) {
    runtime_params.is_super_cap_online = true;
    runtime_params.super_cap_mode = hello_world::power_limiter::kPwrLimitSuperCapNormal;
    runtime_params.z_dummy_measure = cap_ptr_->getRemainingPower();
    runtime_params.p_cap_measure = cap_ptr_->getOutputPower();
  }

  if (is_high_spd_enabled_) {
    runtime_params.p_rfr_max += 800.0f;
    runtime_params.z_rfr_measure = 60.0f;
  }
  pwr_limiter_ptr_->PwrLimitUpdateRuntimeParams(runtime_params);  // 更新运行时参数
  // 进行功率限制
  hello_world::power_limiter::MotorRuntimeParams motor_run_par[4];//(WPY)TODO：补充对于舵电机的功率限制器
  for(uint8_t i = 0; i < 4; i++) {
    motor_run_par[i].iq_measure = wheel_motor_ptr_[i]->curr();
    motor_run_par[i].spd_measure_radps = wheel_motor_ptr_[i]->vel();
    motor_run_par[i].spd_ref_radps = wheel_speed_ref_[i];
    motor_run_par[i].type = hello_world::power_limiter::kMotor3508;
  };
  PwrLimiter::MotorRuntimeParamsList motor_run_par_list = {motor_run_par[0], motor_run_par[1], motor_run_par[2], motor_run_par[3]};
  pwr_limiter_ptr_->PwrLimitCalcSpd(motor_run_par_list, wheel_speed_ref_limited_);
};

void Chassis::calcPwrLimitedCurrentRef() {};

void Chassis::calcWheelCurrentRef()
{
  // 计算每个轮子的期望转速
  // 期望转速由 PID 控制器计算，期望转速与实际转速之间的差距由限幅器控制
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  MultiNodesPid *pid_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = wheel_pid_ptr_[wpis[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to Wheel PID %d is nullptr", wpis[i]);
    // pid_ptr->calc(&wheel_speed_ref_limited_[i], &wheel_speed_fdb_[i], nullptr, &wheel_current_ref_[i]);
    pid_ptr->calc(&wheel_speed_ref_[i], &wheel_speed_fdb_[i], nullptr, &wheel_current_ref_[i]);
  }
};
void Chassis::calcWheelCurrentLimited() {
};

void Chassis::calcSteerVoltageRef()
{
  SteerPidIdx spis[4] = {
      kSteerPidIdxLeftFront,
      kSteerPidIdxLeftRear,
      kSteerPidIdxRightRear,
      kSteerPidIdxRightFront,
  };

    MultiNodesPid *pid_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = steer_pid_ptr_[spis[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to Steer PID %d is nullptr", spis[i]);

    float steer_motor_fdb[2] = {steer_angle_fdb_[i], steer_speed_fdb_[i]}; //TODO滤波处理速度

    pid_ptr->calc(&steer_angle_ref_[i], steer_motor_fdb, nullptr, &steer_voltage_ref_[i]);
  }
}

#pragma endregion

#pragma region 数据重置函数
void Chassis::reset()
{
  pwr_state_ = PwrState::Dead;       ///< 电源状态
  last_pwr_state_ = PwrState::Dead;  ///< 上一电源状态

  // 由 robot 设置的数据
  use_cap_flag_ = false;           ///< 是否使用超级电容
  gyro_dir_ = GyroDir::NotRotate;  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  norm_cmd_.reset();
  rfr_data_ = RfrData();  ///< 底盘 RFR 数据

  working_mode_ = WorkingMode::Depart;       ///< 工作模式
  last_working_mode_ = WorkingMode::Depart;  ///< 上一次工作模式

  // 在 update 函数中更新的数据
  is_power_on_ = false;  ///< 底盘电源是否开启
  // work_tick_ = 0;          ///< 记录底盘模块的运行时间，单位为 ms
  // last_pwr_off_tick_ = 0;  ///< 上一次底盘电源处于关闭状态的时间戳，单位为 ms，实际上是作为上电瞬间的记录

  // 在 runOnWorking 函数中更新的数据
  resetMotorsRef();
  resetCmds();
  // gimbal board fdb data  在 update 函数中更新
  is_gimbal_imu_ready_ = false;  ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  is_all_wheel_online_ = false;  ///< 所有轮电机是否都处于就绪状态
  is_any_wheel_online_ = false;  ///< 任意轮电机是否处于就绪状态
  is_all_steer_online_ = false;  ///< 所有舵电机是否都处于就绪状态
  is_any_steer_online_ = false;  ///< 任意舵电机是否处于就绪状态
 
  theta_i2r_ = 0.0f;  ///< 图传坐标系绕 Z 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  is_high_spd_enabled_ = false;  ///< 是否开启了高速模式 （开启意味着从电容取电）
  cap_remaining_energy_ = 0.0f;  ///< 剩余电容能量百分比，单位 %

  resetMotorsFdb();
  resetPids();  ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnDead()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids();  ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnResurrection()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids();  ///< 重置 PID 控制器参数
};

void Chassis::resetDataOnStandby()
{
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids();      ///< 重置 PID 控制器参数
};

//重置控制指令
void Chassis::resetCmds()
{
  cmd_.reset();       ///< 控制指令，基于图传坐标系
  last_cmd_.reset();  ///< 上一控制周期的控制指令，基于图传坐标系

  rev_head_flag_ = false;       ///< 恢复反转标志位
  // last_rev_head_tick_ = 0;   ///< 上一次转向后退的时间戳
};

//重置轮、舵电机参考数据
void Chassis::resetMotorsRef(){
  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  
  memset(steer_speed_ref_, 0, sizeof(steer_speed_ref_));
  memset(steer_speed_ref_limited_, 0, sizeof(steer_speed_ref_limited_));
  memset(steer_angle_ref_, 0, sizeof(steer_angle_ref_));
  memset(steer_voltage_ref_, 0, sizeof(steer_voltage_ref_));
}

//重置轮、舵电机反馈数据
void Chassis::resetMotorsFdb()
{
  memset(wheel_speed_fdb_, 0, sizeof(wheel_speed_fdb_));
  memset(wheel_current_fdb_, 0, sizeof(wheel_current_fdb_));

  memset(steer_speed_fdb_, 0, sizeof(steer_speed_fdb_));
  memset(steer_angle_fdb_, 0, sizeof(steer_angle_fdb_));
  memset(steer_current_fdb_, 0, sizeof(steer_current_fdb_));
}

void Chassis::resetPids()
{
  for (size_t i = 0; i < kWheelPidNum; i++) {
    wheel_pid_ptr_[i]->reset();
    steer_pid_ptr_[i]->reset();
  }
  follow_omega_pid_ptr_->reset();
};
#pragma endregion

#pragma region 通讯数据设置函数

//将轮、舵电机相关信息补充进本函数
void Chassis::setCommDataMotors(bool working_flag)
{
  // 轮电机根据期望电流输入发送数据
  WheelMotorIdx wmis[4] = {
      kWheelMotorIdxLeftFront,
      kWheelMotorIdxLeftRear,
      kWheelMotorIdxRightRear,
      kWheelMotorIdxRightFront,
  };
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  // 舵电机根据期望电压输入发送数据
  SteerMotorIdx smis[4] = {
      kSteerMotorIdxLeftFront,
      kSteerMotorIdxLeftRear,
      kSteerMotorIdxRightRear,
      kSteerMotorIdxRightFront,
  };
  SteerPidIdx spis[4] = {
      kSteerPidIdxLeftFront,
      kSteerPidIdxLeftRear,
      kSteerPidIdxRightRear,
      kSteerPidIdxRightFront,
  };

  Motor *wheel_motor_ptr = nullptr;
  Motor *steer_motor_ptr = nullptr;
  MultiNodesPid *wheel_pid_ptr = nullptr;
  MultiNodesPid *steer_pid_ptr = nullptr;

  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    WheelPidIdx wpi = wpis[i];
    SteerMotorIdx smi = smis[i];
    SteerPidIdx spi = spis[i];

    wheel_motor_ptr = wheel_motor_ptr_[wmi];
    wheel_pid_ptr = wheel_pid_ptr_[wpi];
    steer_motor_ptr = steer_motor_ptr_[smi];
    steer_pid_ptr = steer_pid_ptr_[spi];

    HW_ASSERT(wheel_motor_ptr != nullptr, "pointer to wheel motor %d is nullptr", wmi);
    HW_ASSERT(steer_motor_ptr != nullptr, "pointer to steer motor %d is nullptr", smi);

    if (!working_flag || wheel_motor_ptr->isOffline()) {
      wheel_pid_ptr->reset();
      wheel_motor_ptr->setInput(0);
    } else {
      // wheel_motor_ptr->setInput(0); //TODO调试
      wheel_motor_ptr->setInput(wheel_current_ref_[wmi]);
    }

    if (!working_flag ||steer_motor_ptr->isOffline()) {
      steer_pid_ptr->reset();
      steer_motor_ptr->setInput(0);
    } else {
      steer_motor_ptr->setInput(steer_voltage_ref_[smi]);
    }
  }
};

void Chassis::setCommDataCap(bool working_flag)
{
  // 电容根据电容充电状态发送数据
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (working_flag && use_cap_flag_) {
    cap_ptr_->enable();
  } else {
    cap_ptr_->disable();
  }
  cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, rfr_data_.current_hp);
};

#pragma endregion

#pragma region 注册函数

void Chassis::registerIkSolver(ChassisIkSolver *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to IK solver is nullptr", ptr);
  ik_solver_ptr_ = ptr;
};

void Chassis::registerWheelMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to wheel motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum, "index of wheel motor out of range", idx);
  wheel_motor_ptr_[idx] = ptr;
};

void Chassis::registerSteerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to steer motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kSteerMotorNum, "index of steer motor out of range", idx);
  steer_motor_ptr_[idx] = ptr;
};

void Chassis::registerYawMotor(Motor *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Yaw motor is nullptr", ptr);
  yaw_motor_ptr_ = ptr;
}

void Chassis::registerWheelPid(MultiNodesPid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to wheel PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelPidNum, "index of wheel PID out of range", idx);
  wheel_pid_ptr_[idx] = ptr;
};

void Chassis::registerSteerPid(MultiNodesPid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to steer PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kSteerPidNum, "index of steer PID out of range", idx);
  steer_pid_ptr_[idx] = ptr;
};

void Chassis::registerFollowOmegaPid(MultiNodesPid *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID is nullptr", ptr);
  follow_omega_pid_ptr_ = ptr;
};

void Chassis::registerPwrLimiter(PwrLimiter *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to PwrLimiter is nullptr", ptr);
  pwr_limiter_ptr_ = ptr;
};

void Chassis::registerCap(Cap *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Capacitor is nullptr", ptr);
  cap_ptr_ = ptr;
};

void Chassis::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/
}  // namespace robot