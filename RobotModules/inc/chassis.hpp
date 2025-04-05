/**
 *******************************************************************************
 * @file      :chassis.hpp
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
#ifndef ROBOT_MODULES_CHASSIS_HPP_
#define ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <stdint.h>

#include "allocator.hpp"
#include "arm_math.h"
#include "chassis_iksolver.hpp"
#include "pid.hpp"
#include "power_limiter.hpp"
#include "ramp.hpp"

#include "motor.hpp"
#include "super_cap.hpp"

#include "module_fsm.hpp"

#include "gimbal_chassis_comm.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

union ChassisState {
  struct {
    float v_x;
    float v_y;
    float w;
  };
  float data[3];
  void reset() {
    v_x = 0;
    v_y = 0;
    w = 0;
  }
  ChassisState operator+(const ChassisState &other) const {
    return {v_x + other.v_x, v_y + other.v_y, w + other.w};
  }

  ChassisState operator-(const ChassisState &other) const {
    return {v_x - other.v_x, v_y - other.v_y, w - other.w};
  }

  ChassisState operator*(float scalar) const {
    return {v_x * scalar, v_y * scalar, w * scalar};
  }

  ChassisState operator+=(const ChassisState &other) {
    v_x += other.v_x;
    v_y += other.v_y;
    w += other.w;
    return *this;
  }

  ChassisState operator-=(const ChassisState &other) {
    v_x -= other.v_x;
    v_y -= other.v_y;
    w -= other.w;
    return *this;
  }

  ChassisState operator*=(float scalar) {
    v_x *= scalar;
    v_y *= scalar;
    w *= scalar;
    return *this;
  }

  friend ChassisState operator*(float scalar, const ChassisState &state);
};

struct ChassisRfrData {
  bool is_rfr_on = false; ///< 裁判系统是否在线
  bool is_pwr_on =
      false; ///< 机器人底盘电源是否开启【裁判系统告知，离线时默认开启】
  uint16_t pwr_limit =
      80; ///< 机器人底盘功率限制【裁判系统告知，离线时采用默认值】
  uint16_t pwr_buffer =
      60; ///< 机器人底盘缓冲能量【裁判系统告知，离线时采用默认值】
  uint16_t current_hp = 100; ///< 底盘血量【裁判系统告知，离线时采用默认值】
};

struct ChassisConfig {
  float normal_trans_vel; ///< 正常平移速度
  float gyro_rot_spd_move;     ///< 平移时小陀螺旋转速度
  float gyro_rot_spd_stand;    ///< 静止时小陀螺旋转速度
  float yaw_sensitivity;  ///< 跟随前馈灵敏度
  float max_trans_vel;    ///< 最大平移速度
  float max_rot_spd;      ///< 最大旋转速度
};

class Chassis : public hello_world::module::ModuleFsm {
public:
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::pid::MultiNodesPid MultiNodesPid;
  typedef hello_world::chassis_ik_solver::ChassisIkSolver ChassisIkSolver;
  typedef hello_world::cap::SuperCap Cap;
  typedef hello_world::power_limiter::PowerLimiter PwrLimiter;
  typedef hello_world::filter::Ramp Ramp;

  typedef hello_world::module::PwrState PwrState;
  typedef hello_world::module::CtrlMode CtrlMode;
  typedef hello_world::module::ManualCtrlSrc ManualCtrlSrc;

  typedef robot::GimbalChassisComm GimbalChassisComm;
  typedef ChassisWorkingMode WorkingMode;

  typedef ChassisState State;
  typedef ChassisRfrData RfrData;
  typedef ChassisConfig Config;

  static std::string WorkingModeToStr(WorkingMode mode) {
    if (mode == Chassis::WorkingMode::Depart)
      return "Depart";
    if (mode == Chassis::WorkingMode::Follow)
      return "Follow";
    if (mode == Chassis::WorkingMode::Gyro)
      return "Gyro";
    return "Unknown";
  };

  enum class GyroDir : int8_t {
    Clockwise = -1,    ///< 顺时针
    Unspecified = 0,   ///< 静止
    AntiClockwise = 1, ///< 逆时针
  };

  enum class GyroMode : int8_t {
    ConstW = 0, ///< 常速旋转
    SinW = 1,   ///< 正弦偏置速度旋转
  };

  enum WheelMotorIdx : uint8_t {
    kWheelMotorIdxLeftFront,  ///< 左前轮电机下标
    kWheelMotorIdxLeftRear,   ///< 左后轮电机下标
    kWheelMotorIdxRightRear,  ///< 右后轮电机下标
    kWheelMotorIdxRightFront, ///< 右前轮电机下标
    kWheelMotorNum,           ///< 轮电机数量
  };

  enum WheelPidIdx : uint8_t {
    kWheelPidIdxLeftFront,  ///< 左前轮 PID
    kWheelPidIdxLeftRear,   ///< 左后轮 PID
    kWheelPidIdxRightRear,  ///< 右后轮 PID
    kWheelPidIdxRightFront, ///< 右前轮 PID
    kWheelPidNum,           ///< 轮PID数量
  };

  enum SteerMotorIdx : uint8_t {
    kSteerMotorIdxLeftFront,  ///< 左前舵电机下标
    kSteerMotorIdxLeftRear,   ///< 左后舵电机下标
    kSteerMotorIdxRightRear,  ///< 右后舵电机下标
    kSteerMotorIdxRightFront, ///< 右前舵电机下标
    kSteerMotorNum,           ///< 舵电机数量
  };

  enum SteerPidIdx : uint8_t {
    kSteerPidIdxLeftFront,  ///< 左前舵 PID
    kSteerPidIdxLeftRear,   ///< 左后舵 PID
    kSteerPidIdxRightRear,  ///< 右后舵 PID
    kSteerPidIdxRightFront, ///< 右前舵 PID
    kSteerPidNum,           ///< 舵PID数量
  };

  Chassis(const Config &config) { config_ = config; };
  ~Chassis() {};

  void update() override;

  void runOnDead() override;
  void runOnResurrection() override;
  void runOnWorking() override;
  void runAlways() override;

  void reset() override;
  void standby() override;

  WorkingMode getWorkingMode() const { return working_mode_; }
  WorkingMode getLastWorkingMode() const { return last_working_mode_; }
  void setWorkingMode(WorkingMode mode) {
    if (working_mode_ != mode) {
      last_working_mode_ = working_mode_;
      working_mode_ = mode;
    }
  }
  void setNormCmd(const State &cmd) { cmd_norm_ = cmd; }
  void setRfrData(const RfrData &data) { rfr_data_ = data; }
  float getThetaI2r(bool rev_gimbal_flag = false,
                    bool actual_head_dir = true) const {
    if (actual_head_dir == true) {
      return theta_i2r_;
    }

    if (rev_chassis_flag_ || rev_gimbal_flag) {
      return hello_world::AngleNormRad(theta_i2r_ + PI);
    } else {
      return theta_i2r_;
    }
  };
  void revChassis() {
      last_rev_chassis_flag_ = rev_chassis_flag_;
      rev_chassis_flag_ = !rev_chassis_flag_;
      last_rev_chassis_tick_ = work_tick_;
  }
  void setGyroDir(GyroDir dir) { gyro_dir_ = dir; }
  void setGyroMode(GyroMode mode) { gyro_mode_ = mode; }
  void setOmegaFeedforward(float omega) { omega_feedforward_ = omega; }
  void setUseCapFlag(bool flag) { use_cap_flag_ = flag; }
  bool getUseCapFlag() const { return use_cap_flag_; }

  void registerRampCmdVx(Ramp *ptr);
  void registerRampCmdVy(Ramp *ptr);
  void registerRampCmdV(Ramp *ptr);
  void registerIkSolver(ChassisIkSolver *ptr);
  void registerWheelMotor(Motor *ptr, int idx);
  void registerSteerMotor(Motor *ptr, int idx);
  void registerYawMotor(Motor *ptr);
  void registerWheelPid(MultiNodesPid *ptr, int idx);
  void registerSteerPid(MultiNodesPid *ptr, int idx);
  void registerFollowOmegaPid(MultiNodesPid *ptr);
  void registerCap(Cap *ptr);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  void registerPwrLimiter(PwrLimiter *ptr);

private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateGimbalBoardData();
  void updateMotorData();
  void updateCapData();
  void updateIsPowerOn();
  void updatePwrState();

  // 工作状态下，获取控制指令的函数
  void calcChassisState();
  void revNormCmd();
  void calcMotorsRef();
  void calcMotorsLimitedRef();
  void calcPwrLimitedCurrentRef();
  void calcWheelCurrentRef();
  void calcWheelCurrentLimited();
  void calcSteerCurrentRef();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetDataOnStandby();
  void resetCmds();
  void resetRuntimeFlags();
  void resetMotorsRef();
  void resetMotorsFdb();
  void resetPids();

  // 设置通讯组件数据函数
  void setCommData(bool is_working) {
    setCommDataMotors(is_working);
    setCommDataCap(is_working);
  };
  void setCommDataMotors(bool is_working);
  void setCommDataCap(bool is_working);

  // 工具函数
  void LinearFilter(const float in, float *last_in, float *out,
                    const float beta = 0.9f) {
    *out = in * beta + *last_in * (1 - beta);
    *last_in = *out;
  }

  // 配置参数
  Config config_;

  // 由 robot 设置的数据
  bool use_cap_flag_ = false; ///< 是否使用超级电容
  GyroDir gyro_dir_ = GyroDir::Unspecified,
          last_gyro_dir_ = GyroDir::Unspecified;
  GyroMode gyro_mode_ = GyroMode::ConstW; ///< 陀螺模式
  ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  State cmd_norm_ = {0};    ///< 原始控制指令，基于图传坐标系
  ChassisRfrData rfr_data_; ///< 底盘 RFR 数据

  WorkingMode working_mode_ = WorkingMode::Depart;      ///< 工作模式
  WorkingMode last_working_mode_ = WorkingMode::Depart; ///< 上一次工作模式

  // 在 update 函数中更新的数据
  bool is_power_on_ = false; ///< 底盘电源是否开启
  uint32_t last_pwr_off_tick_ =
      0; ///< 上一次底盘电源处于关闭状态的时间戳，单位为
  ///< ms，实际上是作为上电瞬间的记录
  uint32_t resurrection_tick_ = 0; ///< 底盘模块复活时间戳

  // 在 runOnWorking 函数中更新的数据
  State cmd_ = {0.0f}; ///< 控制指令，基于图传坐标系
  State cmd_state_ = {0.0f},
        last_cmd_state_ = {0.0f}; ///< 反映底盘实际运动状态的控制指令
  float omega_feedforward_ =
      0; ///< 云台 YAW 轴角速度，用于底盘跟随前馈，单位 rad/s
  float wheel_speed_ref_[4] = {0}; ///< 轮电机的速度参考值 单位 rad/s
  float wheel_speed_ref_limited_[4] = {
      0};                            ///< 轮电机的速度参考值(限幅后) 单位 rad/s
  float wheel_current_ref_[4] = {0}; ///< 轮电机的电流参考值 单位 A [-20, 20]
  float wheel_current_ref_limited_[4] = {0}; ///< 轮电机的电流参考值(限幅后)
                                             ///< 单位 A [-20, 20]
  float steer_speed_ref_[4] = {0};           ///< 舵电机的速度参考值 单位 rad/s
  float steer_angle_ref_[4] = {0};           ///< 舵电机的角度参考值 单位 rad
  float steer_current_ref_[4] = {0}; ///< 舵电机的电流参考值 单位 A [-3.0, 3.0]
  float steer_current_ref_limited_[4] = {
      0}; ///< 舵电机的电流参考值(限幅后) 单位 rad/s
  State chassis_state_ = {0}, last_chassis_state_ = {0}; ///< 底盘状态

  bool rev_chassis_flag_ = false;      ///< 转向后退标志
  bool last_rev_chassis_flag_ = false; ///< 上一次转向后退标志
  uint32_t last_rev_chassis_tick_ = 0; ///< 上一次底盘转向的时间戳

  // gimbal board fdb data  在 update 函数中更新
  bool is_gimbal_imu_ready_ = false; ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  bool is_all_wheel_online_ = false;    ///< 所有轮电机是否都处于就绪状态
  bool is_any_wheel_online_ = false;    ///< 任意轮电机是否处于就绪状态
  float wheel_speed_fdb_[4] = {0};      ///< 轮速反馈数据
  float wheel_current_fdb_[4] = {0};    ///< 轮电流反馈数据
  bool is_all_steer_online_ = false;    ///< 所有舵电机是否都处于就绪状态
  bool is_any_steer_online_ = false;    ///< 任意舵电机是否处于就绪状态
  float steer_speed_fdb_[4] = {0};      ///< 舵速反馈数据
  float last_steer_speed_fdb_[4] = {0}; ///< 上一次舵速度反馈数据
  float steer_angle_fdb_[4] = {0};      ///< 舵角度反馈数据
  float steer_current_fdb_[4] = {0};    ///< 舵电流反馈数据
  float theta_i2r_ =
      0.0f; ///< 图传坐标系绕 Z
            ///< 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  bool is_high_spd_enabled_ =
      false; ///< 是否开启了高速模式 （开启意味着从电容取电）
  float cap_remaining_energy_ = 0.0f; ///< 剩余电容能量百分比，单位 %

  // 各组件指针
  // 无通信功能的组件指针
  ChassisIkSolver *ik_solver_ptr_ = nullptr;               ///< 逆解算器指针
  MultiNodesPid *wheel_pid_ptr_[kWheelPidNum] = {nullptr}; ///< 轮电机 PID 指针
  MultiNodesPid *steer_pid_ptr_[kSteerPidNum] = {nullptr}; ///< 舵电机 PID 指针
  MultiNodesPid *follow_omega_pid_ptr_ = nullptr; ///< 跟随模式下角速度 PID 指针
  PwrLimiter *pwr_limiter_ptr_ = nullptr;
  Ramp *ramp_cmd_vx_ptr_ = nullptr; ///< Vx斜坡滤波指针
  Ramp *ramp_cmd_vy_ptr_ = nullptr; ///< Vy斜坡滤波指针
  Ramp *ramp_cmd_v_ptr_ = nullptr;  ///< V斜坡滤波指针
  // 只接收数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr; ///< 云台底盘通信器指针 只接收数据
  Motor *yaw_motor_ptr_ = nullptr;           ///< 云台电机指针 接收、发送数据
  // 接收、发送数据的组件指针
  Cap *cap_ptr_ = nullptr; ///< 超电指针 接收、发送数据
  Motor *wheel_motor_ptr_[kWheelMotorNum] = {
      nullptr}; ///< 轮电机指针 接收、发送数据 【YAW 只接收数据】
  Motor *steer_motor_ptr_[kSteerMotorNum] = {
      nullptr}; ///< 舵电机指针 接收、发送数据 【YAW 只接收数据】
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

inline ChassisState operator*(float scalar, const ChassisState &cmd) {
  return cmd * scalar;
};

} // namespace robot
#endif /* ROBOT_MODULES_CHASSIS_HPP_ */
