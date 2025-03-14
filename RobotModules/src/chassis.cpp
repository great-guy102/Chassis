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

hello_world::pid::MultiNodesPid::Datas pid_data; // TODO:PID调试数据
// 使用示例：pid_data = pid_ptr->getPidAt(0).datas();
// Chassis::State cmd_state_raw = {0.0f}, cmd_state = {0.0f}; // TODO:调试
namespace robot {
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
void Chassis::update() {
  updateData();
  updatePwrState();
};

void Chassis::updateData() {
  updateWorkTick();
  updateGimbalBoardData();
  updateMotorData();
  updateCapData();
  updateIsPowerOn();
};

/**
 * 更新电源状态
 *
 * 无论处于任何状态，只要掉电就切换到死亡状态。
 * 死亡状态下，如果上电，则切到复活状态；
 * 复活状态下，如果有轮或舵电机上电完毕（底盘已经准备完毕），且云台板 IMU
 * 准备就绪，则切到工作状态； 工作状态下，保持当前状态；
 * 其他状态，认为是死亡状态。
 */
void Chassis::updatePwrState() {
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_power_on_) {
    setPwrState(PwrState::kDead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::kDead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_power_on_) {
      next_state = PwrState::kResurrection;
      resurrection_tick_ = 0;
    }
  } else if (current_state == PwrState::kResurrection) {
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，底盘就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么只要有轮或舵电机上电完毕，就认为底盘就准备好了？
    // 因为底盘逆解不需要所有轮或舵电机都在线也可以运行
    // 在后续正常工作状态下，会对轮或舵电机的状态进行检测
    // 如果有轮或舵电机掉电，会进行专门的处理，比如 pid 清空、can 发无效数据等
    // 3. 所有控制模式都需要 yaw 轴电机的角度，为什么不判断 yaw 轴电机的状态？
    // 因为 yaw 轴电机的状态不影响底盘的运动解算
    // 当 yaw 轴电机离线时，底盘会按照底盘坐标系进行解算（yaw
    // 电机离线后数据清空，默认返回 0，能跑但是疯了，但总比不能跑强） 当 yaw
    // 轴电机上电时，底盘会按照图传坐标系进行解算
    if (is_gimbal_imu_ready_ &&
        (is_any_wheel_online_ || is_any_steer_online_)) {
      resurrection_tick_++;
      // 底盘缓启动
      if (resurrection_tick_ > 2000) {
        next_state = PwrState::kWorking;
      }
    } else {
      resurrection_tick_ = 0;
    }

  } else if (current_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

void Chassis::updateGimbalBoardData() {
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板 IMU 准备就绪，使得可以在无云台板下工作
  HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr",
            gc_comm_ptr_);
  if (gc_comm_ptr_->isOffline()) {
    is_gimbal_imu_ready_ = true;
  } else {
    is_gimbal_imu_ready_ =
        gc_comm_ptr_->main_board_data().gp.is_gimbal_imu_ready;
  }
};

void Chassis::updateMotorData() {
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

    HW_ASSERT(wheel_motor_ptr != nullptr,
              "pointer to wheel motor %d is nullptr", wmi);
    HW_ASSERT(steer_motor_ptr != nullptr,
              "pointer to steer motor %d is nullptr", smi);
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

  HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to Yaw motor is nullptr",
            yaw_motor_ptr_);
  if (yaw_motor_ptr_->isOffline()) {
    theta_i2r_ = 0.0;
  } else {
    theta_i2r_ = -yaw_motor_ptr_->angle();
  }
};

void Chassis::updateCapData() {
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (cap_ptr_->isOffline()) {
    is_high_spd_enabled_ = false;
    cap_remaining_energy_ = 0.0f;
  } else {
    is_high_spd_enabled_ = cap_ptr_->isUsingSuperCap();
    cap_remaining_energy_ = cap_ptr_->getRemainingPower();
  }
};

void Chassis::updateIsPowerOn() {
  is_power_on_ =
      (is_any_wheel_online_ && is_all_steer_online_) || rfr_data_.is_pwr_on;
  if (!is_power_on_) {
    last_pwr_off_tick_ = work_tick_;
  }
};

#pragma endregion

#pragma region 执行任务
void Chassis::runOnDead() { resetDataOnDead(); };

void Chassis::runOnResurrection() { resetDataOnResurrection(); };

void Chassis::runOnWorking() {
  calcChassisState();
  revNormCmd();
  calcMotorsRef();
  calcSteerCurrentRef();
  calcMotorsLimitedRef();
  calcWheelCurrentRef();
  // calcChassisState(); // TODO:调试
};

void Chassis::runAlways() { setCommData(pwr_state_ == PwrState::kWorking); };

void Chassis::standby() {
  resetDataOnStandby();
  setCommData(false);
};
#pragma endregion

#pragma region 工作状态下，获取控制指令的函数
/**
 * @brief 计算舵轮底盘的正运动学解算器
 */
void Chassis::calcChassisState() {
  // 初始化底盘状态
  const float length = 0.334f;
  const float width = 0.334f;
  const float wheel_radius = 66 * 0.001f;
  // 计算轮子到底盘中心的距离平方
  const float r_square = (length * length + width * width) / 4.0f;

  float theta_i2r = 0.0f;
  State chassis_state_raw = {0.0f};

  if (is_all_wheel_online_ && is_all_steer_online_) {
    float steer_angle[4] = {0.0f}, wheel_speed[4] = {0.0f};
    for (size_t i = 0; i < 4; i++) {
      steer_angle[i] = steer_angle_fdb_[i];
      // steer_angle[i] = steer_angle_ref_[i]; // TODO:调试
      wheel_speed[i] = wheel_speed_fdb_[i];
      // wheel_speed[i] = wheel_speed_ref_[i]; // TODO:调试
    }
    theta_i2r = theta_i2r_; // 正解算遵循右手定则

    // 四个轮子的x和y分量速度
    float v_x[4] = {0.0f}, v_y[4] = {0.0f};

    // 计算四个轮子的线速度分量
    for (int i = 0; i < 4; i++) {
      // 将轮子角速度转换为线速度
      float linear_speed = wheel_speed[i] * wheel_radius;
      v_x[i] = linear_speed * cosf(steer_angle[i]);
      v_y[i] = linear_speed * sinf(steer_angle[i]);
    }

    // 左前轮贡献
    chassis_state_raw.v_x += v_x[0];
    chassis_state_raw.v_y += v_y[0];
    chassis_state_raw.w +=
        (-v_y[0] * length + v_x[0] * width) / (2.0f * r_square);

    // 左后轮贡献
    chassis_state_raw.v_x += v_x[1];
    chassis_state_raw.v_y += v_y[1];
    chassis_state_raw.w +=
        (-v_y[1] * (-length) + v_x[1] * width) / (2.0f * r_square);

    // 右后轮贡献
    chassis_state_raw.v_x += v_x[2];
    chassis_state_raw.v_y += v_y[2];
    chassis_state_raw.w +=
        (-v_y[2] * (-length) + v_x[2] * (-width)) / (2.0f * r_square);

    // 右前轮贡献
    chassis_state_raw.v_x += v_x[3];
    chassis_state_raw.v_y += v_y[3];
    chassis_state_raw.w +=
        (-v_y[3] * length + v_x[3] * (-width)) / (2.0f * r_square);

    // 计算平均值
    chassis_state_raw *= 0.25f;
  }

  // 坐标系转换 - 底盘坐标系 -> 云台坐标系
  // 右手系旋转矩阵: R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)]
  float sin_theta = sinf(theta_i2r);
  float cos_theta = cosf(theta_i2r);

  // 计算云台坐标系下的速度
  chassis_state_.v_x =
      chassis_state_raw.v_x * cos_theta - chassis_state_raw.v_y * sin_theta;
  chassis_state_.v_y =
      chassis_state_raw.v_x * sin_theta +
      chassis_state_raw.v_y *
          cos_theta; // 遥控器遥杆向左，指令为负数，指令体系遵循左手系，所以这里取反
  chassis_state_.w = chassis_state_raw.w; // 角速度不受坐标系旋转影响

  // 线性滤波
  float chassis_state_beta = 0.1f;
  LinearFilter(chassis_state_.v_x, &last_chassis_state_.v_x,
               &chassis_state_.v_x, chassis_state_beta);
  LinearFilter(chassis_state_.v_y, &last_chassis_state_.v_y,
               &chassis_state_.v_y, chassis_state_beta);
  LinearFilter(chassis_state_.w, &last_chassis_state_.w, &chassis_state_.w,
               chassis_state_beta);
};

// TODO:调试1
// bool error_flag = true;  // TODO:调试
uint16_t motors_offline_cnt = 0; // TODO:调试
// float error_time = 0.0f; // TODO:调试
void Chassis::revNormCmd() {
  static bool first_follow_flag = true, first_gyro_flag = true;
  State cmd_raw = cmd_norm_;
  State cmd_state_raw = {0.0f};

  switch (working_mode_) {
  case WorkingMode::Depart: {
    // 分离模式不对 cmd_norm_ 进行额外处理
    gyro_dir_ = GyroDir::Unspecified;
    first_follow_flag = true;
    first_gyro_flag = true;
    break;
  }

  case WorkingMode::Gyro: {
    // 陀螺模式下，如果外部没有设置陀螺旋转方向，
    // 则每次进入陀螺模式时，随机选择一个方向
    first_follow_flag = true;

    if (first_gyro_flag) {
      first_gyro_flag = false;
      if (last_gyro_dir_ == GyroDir::Clockwise) {
        gyro_dir_ = GyroDir::AntiClockwise;
      } else if (last_gyro_dir_ == GyroDir::AntiClockwise) {
        gyro_dir_ = GyroDir::Clockwise;
      }
    } else {
      gyro_dir_ = last_gyro_dir_;
    }

    // if (gyro_dir_ == GyroDir::Unspecified) {
    //   if (last_gyro_dir_ == GyroDir::Clockwise) {
    //     gyro_dir_ = GyroDir::AntiClockwise;
    //   } else if (last_gyro_dir_ == GyroDir::AntiClockwise) {
    //     gyro_dir_ = GyroDir::Clockwise;
    //   }
    // }
    // 小陀螺模式下，旋转分量为定值
    cmd_raw.w = config_.gyro_rot_spd * (int8_t)gyro_dir_;
    last_gyro_dir_ = gyro_dir_;
    break;
  }

  case WorkingMode::Follow: {
    first_gyro_flag = true;
    // TODO：掉头模式
    // //  在转头过程中，底盘不响应跟随转动指令
    // if (work_tick_ - last_rev_head_tick_ < 800) {
    //   break;
    // }

    // 跟随模式下，更新跟随目标
    float theta_fdb = theta_i2r_;
    static float theta_ref = 0.0f;

    // TODO：跟随模式云台前馈记录
    // float omega_feedforward = config_.yaw_sensitivity * omega_feedforward_;
    // follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, &omega_feedforward,
    //                             &cmd_raw.w);

    if (first_follow_flag) {
      first_follow_flag = false;
      if (fabs(theta_fdb) < PI / 2) {
        theta_ref = 0.0f;
      } else {
        theta_ref = PI;
      }
    }

    follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, nullptr, &cmd_raw.w);
    pid_data = follow_omega_pid_ptr_->getPidAt(0).datas();
    // 解决跟随模式的低速底盘抖动问题
    if ((cmd_raw.v_x * cmd_raw.v_x + cmd_raw.v_y * cmd_raw.v_y) >= 0.000001f) {
      float w_max = fabsf(cmd_raw.w);
      if ((cmd_raw.v_x * cmd_raw.v_x + cmd_raw.v_y * cmd_raw.v_y) <= 0.0001f) {
        w_max = sqrtf(cmd_raw.v_x * cmd_raw.v_x + cmd_raw.v_y * cmd_raw.v_y);
      } else if ((cmd_raw.v_x * cmd_raw.v_x + cmd_raw.v_y * cmd_raw.v_y) <=
                 0.01f) {
        w_max = 10.0f * sqrtf(cmd_raw.v_x * cmd_raw.v_x +
                              cmd_raw.v_y * cmd_raw.v_y) -
                0.09f;
      }
      cmd_raw.w = hello_world::Bound(cmd_raw.w, -w_max, w_max);
      if (fabs(theta_fdb) < (PI / 90.0f)) {
        cmd_raw.w = 0.0f;
      }
    }

    // TODO:大小死区处理,优化低速震荡问题
    // const float theta_dead_upper = PI / 24.0f;
    // const float theta_dead_lower = PI / 72.0f;
    // static float theta_dead_limit = theta_dead_upper;
    // static bool dead_cross_flag = false;
    // if (!dead_cross_flag && fabs(theta_fdb) > theta_dead_upper) {
    //   theta_dead_limit = theta_dead_lower;
    //   dead_cross_flag = true;
    // } else if (dead_cross_flag && fabs(theta_fdb) < theta_dead_lower) {
    //   theta_dead_limit = theta_dead_upper;
    //   dead_cross_flag = false;
    // }
    // if (fabs(theta_fdb) < theta_dead_limit) {
    //   cmd_raw.w = 0.0f;
    // }

    break;
  }
  default: {
    break;
  }
  }

  cmd_state_raw.v_x = cmd_raw.v_x * config_.normal_trans_vel;
  cmd_state_raw.v_y = cmd_raw.v_y * config_.normal_trans_vel;
  // TODO：预期限幅式斜坡优化
  if (is_all_wheel_online_ && is_all_steer_online_) {
    // cmd_state_raw.v_x =
    //     hello_world::Bound(cmd_state_raw.v_x, chassis_state_.v_x - 0.5f,
    //                        chassis_state_.v_x + 0.5f);
    // cmd_state_raw.v_y =
    //     hello_world::Bound(cmd_state_raw.v_y, chassis_state_.v_y - 0.5f,
    //                        chassis_state_.v_y + 0.5f);
    float cmd_state_raw_v = 0.0f, chassis_state_v = 0.0f;
    cmd_state_raw_v = sqrt(cmd_state_raw.v_x * cmd_state_raw.v_x +
                           cmd_state_raw.v_y * cmd_state_raw.v_y);
    chassis_state_v = sqrt(chassis_state_.v_x * chassis_state_.v_x +
                           chassis_state_.v_y * chassis_state_.v_y);
    cmd_state_raw_v = hello_world::Bound(
        cmd_state_raw_v, chassis_state_v - 0.5f, chassis_state_v + 0.5f);

    float cmd_state_raw_atan2_deg = 0.0f, cmd_state_raw_atan2_rad = 0.0f,
          cmd_state_raw_sin = 0.0f, cmd_state_raw_cos = 0.0f;
    arm_atan2_f32(cmd_state_raw.v_y, cmd_state_raw.v_x,
                  &cmd_state_raw_atan2_rad);
    cmd_state_raw_atan2_deg = hello_world::Rad2Deg(cmd_state_raw_atan2_rad);
    arm_sin_cos_f32(cmd_state_raw_atan2_deg, &cmd_state_raw_sin,
                    &cmd_state_raw_cos);
    cmd_state_raw.v_x = cmd_state_raw_v * cmd_state_raw_cos;
    cmd_state_raw.v_y = cmd_state_raw_v * cmd_state_raw_sin;

    ramp_cmd_vx_ptr_->calc(&(cmd_state_raw.v_x), &(cmd_state_.v_x));
    ramp_cmd_vy_ptr_->calc(&(cmd_state_raw.v_y), &(cmd_state_.v_y));
    // cmd_state_.v_x = cmd_state_raw.v_x; //TODO:调试
    // cmd_state_.v_y = cmd_state_raw.v_y; //TODO:调试
  } else {
    cmd_state_raw.reset();
    cmd_state_.reset();
    ramp_cmd_vx_ptr_->reset();
    ramp_cmd_vy_ptr_->reset();
    motors_offline_cnt++;
  }

  // 电机掉线测试代码
  //  if (!(is_all_wheel_online_ && is_all_steer_online_) && !error_flag) {
  //    cmd_state_.reset();
  //    cnt++;
  //    error_flag = true;
  //    error_time = work_tick_;
  //  } else if ((work_tick_ - error_time) > 200 && error_flag) {
  //    error_flag = false;
  //  }

  cmd_state_.v_x = hello_world::Bound(cmd_state_.v_x, -config_.max_trans_vel,
                                      config_.max_trans_vel);
  cmd_state_.v_y = hello_world::Bound(cmd_state_.v_y, -config_.max_trans_vel,
                                      config_.max_trans_vel);
  cmd_state_.w =
      hello_world::Bound(cmd_raw.w, -config_.max_rot_spd, config_.max_rot_spd);

  // 必要的设置函数，作为Chassis::cmd_的唯一赋值接口
  float cmd_state_beta = 0.9f;
  LinearFilter(cmd_state_.v_x, &last_cmd_state_.v_x, &cmd_.v_x, cmd_state_beta);
  LinearFilter(cmd_state_.v_y, &last_cmd_state_.v_y, &cmd_.v_y, cmd_state_beta);
  LinearFilter(cmd_state_.w, &last_cmd_state_.w, &cmd_.w, cmd_state_beta);
};

void Chassis::calcMotorsRef() {
  // 底盘坐标系下，x轴正方向为底盘正前方，y轴正方向为底盘正左方，z轴正方向为底盘正上方
  // 轮子顺序按照象限顺序进行编号：左前，左后，右后，右前
  HW_ASSERT(ik_solver_ptr_ != nullptr, "pointer to IK solver is nullptr",
            ik_solver_ptr_);
  hello_world::chassis_ik_solver::MoveVec move_vec(cmd_.v_x, cmd_.v_y, cmd_.w);
  float theta_i2r = theta_i2r_;
  ik_solver_ptr_->solve(move_vec, theta_i2r, steer_angle_fdb_);
  ik_solver_ptr_->getRotSpdAll(wheel_speed_ref_);
  ik_solver_ptr_->getThetaVelRefAll(steer_angle_ref_);
  // TODO（WPY）:舵电机模型调试用
  //  for(uint8_t i = 0; i < 4; i++) {
  //    // steer_angle_ref_[i] += 0.03f * cmd_.v_x;
  //    // steer_angle_ref_[i] = hello_world::NormPeriodData(-PI, PI,
  //    steer_angle_ref_[i]); steer_angle_ref_[i] = 2 * M_PI * cmd_.v_x;
  //  }
}

void Chassis::calcSteerCurrentRef() {
  SteerPidIdx spis[4] = {
      kSteerPidIdxLeftFront,
      kSteerPidIdxLeftRear,
      kSteerPidIdxRightRear,
      kSteerPidIdxRightFront,
  };

  MultiNodesPid *pid_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = steer_pid_ptr_[spis[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to Steer PID %d is nullptr",
              spis[i]);

    LinearFilter(steer_speed_fdb_[i], &last_steer_speed_fdb_[i],
                 &steer_speed_fdb_[i], 0.1f);
    float steer_motor_fdb[2] = {steer_angle_fdb_[i],
                                steer_speed_fdb_[i]}; // TODO：滤波处理速度
    pid_ptr->calc(&steer_angle_ref_[i], steer_motor_fdb, nullptr,
                  &steer_current_ref_[i]);
  }
}

float rfr_pwr = 0.0f; // TODO:功限调试
void Chassis::calcMotorsLimitedRef() {
  float up_ref = 120.0f;
  if (working_mode_ == WorkingMode::Gyro) {
    up_ref = 60.0f;
  } else {
    up_ref = 120.0f;
  }
  hello_world::power_limiter::PowerLimiterRuntimeParams runtime_params = {
      .p_ref_max =
          up_ref +
          static_cast<float>(
              rfr_data_.pwr_limit), // 60.0f,//1.2f * rfr_data_.pwr_limit
      .p_referee_max = static_cast<float>(rfr_data_.pwr_limit),
      .p_ref_min = 0.8f * rfr_data_.pwr_limit,
      .remaining_energy = static_cast<float>(rfr_data_.pwr_buffer),
      .energy_converge = 50.0f,
      .p_slope = 2.0f,
      .danger_energy = 5.0f,
  };

  if (!cap_ptr_->isOffline()) {
    if (use_cap_flag_ == true) {
      runtime_params.remaining_energy = cap_ptr_->getRemainingPower();
      runtime_params.energy_converge = 30.0f;
    } else {
      runtime_params.remaining_energy = cap_ptr_->getRemainingPower();
      runtime_params.energy_converge = 50.0f;
    }
  } else {
    runtime_params.remaining_energy = static_cast<float>(rfr_data_.pwr_buffer);
    runtime_params.energy_converge = 20.0f;
  }
  pwr_limiter_ptr_->updateSteeringModel(steer_speed_fdb_, steer_current_ref_,
                                        nullptr);
  pwr_limiter_ptr_->updateWheelModel(wheel_speed_ref_, wheel_speed_fdb_,
                                     nullptr, nullptr);
  pwr_limiter_ptr_->calc(runtime_params, wheel_speed_ref_limited_,
                         steer_current_ref_limited_); // 更新运行时参数
};

void Chassis::calcWheelCurrentRef() {
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
    HW_ASSERT(pid_ptr != nullptr, "pointer to Wheel PID %d is nullptr",
              wpis[i]);
    // pid_ptr->calc(&wheel_speed_ref_[i], &wheel_speed_fdb_[i], nullptr,
    //               &wheel_current_ref_[i]); // TODO: 功限调试
    pid_ptr->calc(&wheel_speed_ref_limited_[i], &wheel_speed_fdb_[i], nullptr,
                  &wheel_current_ref_limited_[i]);
  }
};

#pragma endregion

#pragma region 数据重置函数
void Chassis::reset() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  // 由 robot 设置的数据
  use_cap_flag_ = false; ///< 是否使用超级电容
  gyro_dir_ =
      GyroDir::Unspecified; ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  cmd_norm_.reset();
  rfr_data_ = RfrData(); ///< 底盘 RFR 数据

  working_mode_ = WorkingMode::Depart;      ///< 工作模式
  last_working_mode_ = WorkingMode::Depart; ///< 上一次工作模式

  // 在 update 函数中更新的数据
  is_power_on_ = false; ///< 底盘电源是否开启
  // work_tick_ = 0;          ///< 记录底盘模块的运行时间，单位为 ms
  // last_pwr_off_tick_ = 0;  ///< 上一次底盘电源处于关闭状态的时间戳，单位为
  // ms，实际上是作为上电瞬间的记录

  // 在 runOnWorking 函数中更新的数据
  // gimbal board fdb data  在 update 函数中更新
  is_gimbal_imu_ready_ = false; ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  is_all_wheel_online_ = false; ///< 所有轮电机是否都处于就绪状态
  is_any_wheel_online_ = false; ///< 任意轮电机是否处于就绪状态
  is_all_steer_online_ = false; ///< 所有舵电机是否都处于就绪状态
  is_any_steer_online_ = false; ///< 任意舵电机是否处于就绪状态

  theta_i2r_ = 0.0f; ///< 图传坐标系绕 Z
                     ///< 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  is_high_spd_enabled_ = false; ///< 是否开启了高速模式 （开启意味着从电容取电）
  cap_remaining_energy_ = 0.0f; ///< 剩余电容能量百分比，单位 %

  resetMotorsRef();
  resetCmds();
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnDead() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnResurrection() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};

void Chassis::resetDataOnStandby() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};

// 重置控制指令
void Chassis::resetCmds() {
  cmd_.reset();            ///< 控制指令，基于图传坐标系
  cmd_state_.reset();      ///< 上一控制周期的控制指令，基于图传坐标系
  last_cmd_state_.reset(); ///< 上一控制周期的控制指令，基于图传坐标系

  rev_head_flag_ = false;  ///< 恢复反转标志位
  last_rev_head_tick_ = 0; ///< 上一次转向后退的时间戳
};

// 重置轮、舵电机参考数据
void Chassis::resetMotorsRef() {
  memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  memset(wheel_current_ref_limited_, 0, sizeof(wheel_current_ref_limited_));

  memset(steer_speed_ref_, 0, sizeof(steer_speed_ref_));
  memset(steer_angle_ref_, 0, sizeof(steer_angle_ref_));
  memset(steer_current_ref_, 0, sizeof(steer_current_ref_));
  memset(steer_current_ref_limited_, 0, sizeof(steer_current_ref_limited_));
}

// 重置轮、舵电机反馈数据
void Chassis::resetMotorsFdb() {
  memset(wheel_speed_fdb_, 0, sizeof(wheel_speed_fdb_));
  memset(wheel_current_fdb_, 0, sizeof(wheel_current_fdb_));

  memset(steer_speed_fdb_, 0, sizeof(steer_speed_fdb_));
  memset(last_steer_speed_fdb_, 0, sizeof(last_steer_speed_fdb_));
  memset(steer_angle_fdb_, 0, sizeof(steer_angle_fdb_));
  memset(steer_current_fdb_, 0, sizeof(steer_current_fdb_));
}

void Chassis::resetPids() {
  for (size_t i = 0; i < kWheelPidNum; i++) {
    wheel_pid_ptr_[i]->reset();
    steer_pid_ptr_[i]->reset();
  }
  follow_omega_pid_ptr_->reset();
};
#pragma endregion

#pragma region 通讯数据设置函数

// 将轮、舵电机相关信息补充进本函数
void Chassis::setCommDataMotors(bool working_flag) {
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

    HW_ASSERT(wheel_motor_ptr != nullptr,
              "pointer to wheel motor %d is nullptr", wmi);
    HW_ASSERT(steer_motor_ptr != nullptr,
              "pointer to steer motor %d is nullptr", smi);

    if (!working_flag || wheel_motor_ptr->isOffline()) {
      wheel_pid_ptr->reset();
      wheel_motor_ptr->setInput(0);
    } else {
      // wheel_motor_ptr->setInput(0); // TODO：调试
      // wheel_motor_ptr->setInput(wheel_current_ref_[wmi]); // TODO：功限调试
      wheel_motor_ptr->setInput(wheel_current_ref_limited_[wmi]);
    }

    if (!working_flag || steer_motor_ptr->isOffline()) {
      steer_pid_ptr->reset();
      steer_motor_ptr->setInput(0);
    } else {
      // steer_motor_ptr->setInput(0); // TODO调试
      // steer_motor_ptr->setInput(steer_current_ref_[smi]); // TODO：功限调试
      steer_motor_ptr->setInput(steer_current_ref_limited_[smi]);
    }
  }
};

void Chassis::setCommDataCap(bool working_flag = true) {
  // 电容根据电容充电状态发送数据
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (pwr_state_ != PwrState::kWorking ||
      (rfr_data_.is_rfr_on && !rfr_data_.is_pwr_on)) {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, 0);
  } else {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit,
                         rfr_data_.current_hp);
  }
};

#pragma endregion

#pragma region 注册函数

void Chassis::registerIkSolver(ChassisIkSolver *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to IK solver is nullptr", ptr);
  ik_solver_ptr_ = ptr;
};

void Chassis::registerWheelMotor(Motor *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to wheel motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum,
            "index of wheel motor out of range", idx);
  wheel_motor_ptr_[idx] = ptr;
};

void Chassis::registerSteerMotor(Motor *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to steer motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kSteerMotorNum,
            "index of steer motor out of range", idx);
  steer_motor_ptr_[idx] = ptr;
};

void Chassis::registerYawMotor(Motor *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to Yaw motor is nullptr", ptr);
  yaw_motor_ptr_ = ptr;
}

void Chassis::registerWheelPid(MultiNodesPid *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to wheel PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelPidNum, "index of wheel PID out of range",
            idx);
  wheel_pid_ptr_[idx] = ptr;
};

void Chassis::registerSteerPid(MultiNodesPid *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to steer PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kSteerPidNum, "index of steer PID out of range",
            idx);
  steer_pid_ptr_[idx] = ptr;
};

void Chassis::registerFollowOmegaPid(MultiNodesPid *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to PID is nullptr", ptr);
  follow_omega_pid_ptr_ = ptr;
};

void Chassis::registerPwrLimiter(PwrLimiter *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to PwrLimiter is nullptr", ptr);
  pwr_limiter_ptr_ = ptr;
};
void Chassis::registerRampCmdVx(Ramp *ptr) {
  HW_ASSERT(ptr != nullptr, "RampCmdVx pointer is null", ptr);
  ramp_cmd_vx_ptr_ = ptr;
};
void Chassis::registerRampCmdVy(Ramp *ptr) {
  HW_ASSERT(ptr != nullptr, "RampCmdVy pointer is null", ptr);
  ramp_cmd_vy_ptr_ = ptr;
};
void Chassis::registerRampCmdV(Ramp *ptr) {
  HW_ASSERT(ptr != nullptr, "RampCmdVw pointer is null", ptr);
  ramp_cmd_v_ptr_ = ptr;
};
void Chassis::registerCap(Cap *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to Capacitor is nullptr", ptr);
  cap_ptr_ = ptr;
};

void Chassis::registerGimbalChassisComm(GimbalChassisComm *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/
} // namespace robot