/**
 *******************************************************************************
 * @file      :ui_drawer.hpp
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
#ifndef CHASSIS_ROBOT_MODULE_UI_DRAWER_HPP_
#define CHASSIS_ROBOT_MODULE_UI_DRAWER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"
#include "module_fsm.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

#include "rfr_encoder.hpp"
#include "rfr_official_pkgs.hpp"

#include "module_state.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class UiDrawer {
public:
  typedef hello_world::referee::GraphicOperation GraphicOperation;
  typedef hello_world::referee::ids::RobotId RobotId;
  typedef hello_world::referee::RfrEncoder RfrEncoder;
  typedef hello_world::referee::String String;
  typedef hello_world::referee::GraphicLayer GraphicLayer;
  typedef hello_world::referee::String::Color StringColor;

  typedef hello_world::module::CtrlMode FsmCtrlMode;
  typedef hello_world::module::ManualCtrlSrc FsmManualCtrlSrc;
  typedef hello_world::module::PwrState FsmWorkState;
  typedef hello_world::module::PwrState PwrState;
  typedef hello_world::module::Fric::WorkingMode ShooterWorkingMode;

  typedef robot::Chassis::WorkingMode ChassisWorkingMode;
  typedef robot::Gimbal::WorkingMode GimbalWorkingMode;

  enum StaticUiIdx {
    kSuiDelAll = 0,
    kSuiChassisStatusTitle,
    kSuiGimbalStatusTitle,
    kSuiShooterStatusTitle,
    kSuiMotionModuleOnlineTitle,
    kSuiShootModuleOnlineTitle,
    kSuiPkgGroup1, // TODO：瞄准距离参考线
    kSuiPkgGroup2, // 自瞄范围参考框
    kSuiPkgNum,
  };

  enum DynamicUiIdx {
    kDuiChassisStatusContent,
    kDuiGimbalStatusContent,
    kDuiShooterStatusContent,
    kDuiMotionModuleOnlineContent,
    kDuiShootModuleOnlineContent,
    kDuiPkgGroup1,
    kDuiPkgGroup2,
    kDuiPkgNum,
  };

  static constexpr size_t kNumAllPkgs = (size_t)kDuiPkgNum + (size_t)kSuiPkgNum;

  UiDrawer() {};
  ~UiDrawer() {};

  void refresh() {
    ui_idx_ = 0;
    n_added_ = 0;
  };
  bool encode(uint8_t *data_ptr, size_t &data_len);

#pragma region 接口函数
  // robot
  void setSenderId(RobotId id) { sender_id_ = id; }

  // chassis
  void setIsWheelMotorsOnline(bool is_online) {
    if (is_online != is_wheel_motors_online_) {
      last_is_wheel_motors_online_ = is_wheel_motors_online_;
      is_wheel_motors_online_ = is_online;
    }
  }
  void setIsSteerMotorsOnline(bool is_online) {
    if (is_online != is_steer_motors_online_) {
      last_is_steer_motors_online_ = is_steer_motors_online_;
      is_steer_motors_online_ = is_online;
    }
  }
  void setChassisThetaI2r(float theta_i2r) { theta_i2r_ = theta_i2r; }
  void setChassisWorkState(FsmWorkState state) {
    if (state != chassis_work_state_) {
      last_chassis_work_state_ = chassis_work_state_;
      chassis_work_state_ = state;
    }
  }
  void setChassisCtrlMode(FsmCtrlMode mode) {
    if (mode != chassis_ctrl_mode_) {
      last_chassis_ctrl_mode_ = chassis_ctrl_mode_;
      chassis_ctrl_mode_ = mode;
    }
  }
  void setChassisManualCtrlSrc(FsmManualCtrlSrc src) {
    if (src != chassis_manual_ctrl_src_) {
      last_chassis_manual_ctrl_src_ = chassis_manual_ctrl_src_;
      chassis_manual_ctrl_src_ = src;
    }
  }
  void setChassisWorkingMode(ChassisWorkingMode mode) {
    if (mode != chassis_working_mode_) {
      last_chassis_working_mode_ = chassis_working_mode_;
      chassis_working_mode_ = mode;
    }
  }

  // gimbal
  void setIsGimbalMotorsOnline(bool is_online) {
    if (is_online != is_gimbal_motors_online_) {
      last_is_gimbal_motors_online_ = is_gimbal_motors_online_;
      is_gimbal_motors_online_ = is_online;
    }
  }
  void setGimbalJointAngPitchFdb(float pitch) {
    gimbal_joint_angle_pitch_fdb_ = pitch;
  }
  void setGimbalJointAngYawFdb(float yaw) { gimbal_joint_angle_yaw_fdb_ = yaw; }
  void setGimbalWorkState(FsmWorkState state) {
    if (state != gimbal_work_state_) {
      last_gimbal_work_state_ = gimbal_work_state_;
      gimbal_work_state_ = state;
    }
  }
  void setGimbalCtrlMode(FsmCtrlMode mode) {
    if (mode != gimbal_ctrl_mode_) {
      last_gimbal_ctrl_mode_ = gimbal_ctrl_mode_;
      gimbal_ctrl_mode_ = mode;
    }
  }
  void setGimbalManualCtrlSrc(FsmManualCtrlSrc src) {
    if (src != gimbal_manual_ctrl_src_) {
      last_gimbal_manual_ctrl_src_ = gimbal_manual_ctrl_src_;
      gimbal_manual_ctrl_src_ = src;
    }
  }
  void setGimbalWorkingMode(GimbalWorkingMode mode) {
    if (mode != gimbal_working_mode_) {
      last_gimbal_working_mode_ = gimbal_working_mode_;
      gimbal_working_mode_ = mode;
    }
  }

  // shooter
  void setIsShooterMotorsOnline(bool is_online) {
    if (is_online != is_shooter_motors_online_) {
      last_is_shooter_motors_online_ = is_shooter_motors_online_;
      is_shooter_motors_online_ = is_online;
    }
  }
  void setShooterStuckFlag(bool flag) {
    if (flag != shooter_stuck_flag_) {
      last_shooter_stuck_flag_ = shooter_stuck_flag_;
      shooter_stuck_flag_ = flag;
    }
  }
  void setFeedStuckStatus(uint8_t status) {
    if (status != feed_stuck_status_) {
      last_feed_stuck_status_ = feed_stuck_status_;
      feed_stuck_status_ = status;
    }
  }
  void setHeat(float heat) { heat_ = heat; }
  void setHeatLimit(float limit) { heat_limit_ = limit; }
  void setShooterWorkState(FsmWorkState state) {
    if (state != shooter_work_state_) {
      last_shooter_work_state_ = shooter_work_state_;
      shooter_work_state_ = state;
    }
  }
  void setShooterCtrlMode(FsmCtrlMode mode) {
    if (mode != shooter_ctrl_mode_) {
      last_shooter_ctrl_mode_ = shooter_ctrl_mode_;
      shooter_ctrl_mode_ = mode;
    }
  }
  void setShooterManualCtrlSrc(FsmManualCtrlSrc src) {
    if (src != shooter_manual_ctrl_src_) {
      last_shooter_manual_ctrl_src_ = shooter_manual_ctrl_src_;
      shooter_manual_ctrl_src_ = src;
    }
  }
  void setShooterWorkingMode(ShooterWorkingMode mode) {
    if (mode != shooter_working_mode_) {
      last_shooter_working_mode_ = shooter_working_mode_;
      shooter_working_mode_ = mode;
    }
  }

  // cap
  void setCapPwrPercent(float percent) { cap_pwr_percent_ = percent; }

  // vision
  void setIsVisionOnline(bool is_online) {
    if (is_online != is_vision_online_) {
      last_is_vision_online_ = is_vision_online_;
      is_vision_online_ = is_online;
    }
  }
  void setIsVisionValid(bool is_vision_valid) {
    is_vision_valid_ = is_vision_valid;
  }
  void setVisTgtX(uint16_t x, bool valid) { vis_tgt_x_ = valid ? x : -1; }
  void setVisTgtY(uint16_t y, bool valid) { vis_tgt_y_ = valid ? y : -1; }

  // hurt
  void setIsArmorHit(bool is_armor_hit) { is_armor_hit_ = is_armor_hit; }
  void setArmorIdHit(uint8_t armor_id_hit) { armor_id_hit_ = armor_id_hit; }

#pragma endregion

private:
#pragma region 编码函数
  template <typename T>
  bool encodePkg(uint8_t *data_ptr, size_t &data_len, GraphicOperation opt,
                 T &pkg) {
    pkg.setSenderId(static_cast<uint16_t>(sender_id_));
    return encoder_.encodeFrame(&pkg, data_ptr, &data_len);
  };

  bool encodeString(uint8_t *data_ptr, size_t &data_len, GraphicOperation opt,
                    String &g, std::string &str) {
    g.setOperation(opt);
    hello_world::referee::InterGraphicStringPackage pkg;
    pkg.setStrintg(g, str);
    return encodePkg(data_ptr, data_len, opt, pkg);
  };
  // 汇总
  bool encodeStaticUi(uint8_t *data_ptr, size_t &data_len, GraphicOperation opt,
                      StaticUiIdx idx);
  bool encodeDynamicUi(uint8_t *data_ptr, size_t &data_len,
                       GraphicOperation opt, DynamicUiIdx idx);
  bool encodeDelAll(uint8_t *data_ptr, size_t &data_len);
  // UI组
  bool encodeStaticPkgGroup1(uint8_t *data_ptr, size_t &data_len,
                             GraphicOperation opt);
  bool encodeStaticPkgGroup2(uint8_t *data_ptr, size_t &data_len,
                             GraphicOperation opt);
  bool encodeDynaUiPkgGroup1(uint8_t *data_ptr, size_t &data_len,
                             GraphicOperation opt);
  bool encodeDynaUiPkgGroup2(uint8_t *data_ptr, size_t &data_len,
                             GraphicOperation opt);
  // WorkState
  bool encodeChassisWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                   GraphicOperation opt);
  bool encodeChassisWorkStateContent(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt);
  bool encodeGimbalWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                  GraphicOperation opt);
  bool encodeGimbalWorkStateContent(uint8_t *data_ptr, size_t &data_len,
                                    GraphicOperation opt);
  bool encodeShooterWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                   GraphicOperation opt);
  bool encodeShooterWorkStateContent(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt);
  // Online
  bool encodeMotionModuleOnlineTitle(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt);
  bool encodeMotionModuleOnlineContent(uint8_t *data_ptr, size_t &data_len,
                                       GraphicOperation opt);
  bool encodeShootModuleOnlineTitle(uint8_t *data_ptr, size_t &data_len,
                                    GraphicOperation opt);
  bool encodeShootModuleOnlineContent(uint8_t *data_ptr, size_t &data_len,
                                      GraphicOperation opt);
#pragma endregion

#pragma region UI生成函数
  void genChassisDir(hello_world::referee::Arc &g_head,
                     hello_world::referee::Arc &g_tail);
  void genChassisPassLineLeft(hello_world::referee::StraightLine &g);
  void genChassisPassLineRight(hello_world::referee::StraightLine &g);

  void genGimbalJointAngPitchFdb(hello_world::referee::FloatingNumber &g);
  void genGimbalJointAngYawFdb(hello_world::referee::FloatingNumber &g);
  void genGimbalPassSafe(hello_world::referee::Circle &g, bool is_safe);

  void genShooterHeat(hello_world::referee::Arc &g);

  void genCapPwrPercent(hello_world::referee::Rectangle &g_rect,
                        hello_world::referee::FloatingNumber &g_num);

  void genVisTgt(hello_world::referee::Circle &g);
  void genVisionbox(hello_world::referee::Rectangle &g_rect);

  void genArmorHit(hello_world::referee::Arc &g_hit);
#pragma endregion
  // encode
  size_t ui_idx_ = 0;
  size_t n_added_ = 0;
  RobotId sender_id_ = RobotId::kBlueStandard3;
  RfrEncoder encoder_;

  // var for chassis
  bool is_wheel_motors_online_ = false,
       last_is_wheel_motors_online_ = false; ///< 轮电机是否全部在线
  bool is_steer_motors_online_ = false,
       last_is_steer_motors_online_ = false; ///< 舵电机是否全部在线
  float theta_i2r_ = 0.0f;
  FsmWorkState chassis_work_state_ = FsmWorkState::kDead;
  FsmWorkState last_chassis_work_state_ = FsmWorkState::kDead;
  FsmCtrlMode chassis_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmCtrlMode last_chassis_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmManualCtrlSrc chassis_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  FsmManualCtrlSrc last_chassis_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  ChassisWorkingMode chassis_working_mode_ = ChassisWorkingMode::Depart;
  ChassisWorkingMode last_chassis_working_mode_ = ChassisWorkingMode::Depart;

  // var for gimbal
  bool is_gimbal_motors_online_ = false,
       last_is_gimbal_motors_online_ = false; ///< 云台是否在线
  float gimbal_joint_angle_pitch_fdb_ = 0.0f,
        gimbal_joint_angle_yaw_fdb_ = 0.0f;
  FsmWorkState gimbal_work_state_ = FsmWorkState::kDead;
  FsmWorkState last_gimbal_work_state_ = FsmWorkState::kDead;
  FsmCtrlMode gimbal_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmCtrlMode last_gimbal_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmManualCtrlSrc gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  FsmManualCtrlSrc last_gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  GimbalWorkingMode gimbal_working_mode_ = GimbalWorkingMode::Normal;
  GimbalWorkingMode last_gimbal_working_mode_ = GimbalWorkingMode::Normal;

  // var for shooter
  bool is_shooter_motors_online_ = false,
       last_is_shooter_motors_online_ = false; ///< 发射电机是否在线
  bool shooter_stuck_flag_ = false,
       last_shooter_stuck_flag_ = false;                   // 记录卡弹状态
  uint8_t feed_stuck_status_ = 0, last_feed_stuck_status_; // 记录卡弹状态
  float heat_ = 0, heat_limit_ = 100;
  FsmWorkState shooter_work_state_ = FsmWorkState::kDead;
  FsmWorkState last_shooter_work_state_ = FsmWorkState::kDead;
  FsmCtrlMode shooter_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmCtrlMode last_shooter_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmManualCtrlSrc shooter_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  FsmManualCtrlSrc last_shooter_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  ShooterWorkingMode shooter_working_mode_ = ShooterWorkingMode::kShoot;
  ShooterWorkingMode last_shooter_working_mode_ = ShooterWorkingMode::kShoot;

  // var for super capacitor
  float cap_pwr_percent_ = 0;

  // var for vision
  bool is_vision_online_ = false,
       last_is_vision_online_ = false; ///< 视觉是否在线
  bool is_vision_valid_ = false;       // 视觉是否瞄到
  int16_t vis_tgt_x_ = 0; ///< 视觉瞄准目标的 x 坐标，单位为像素，负数为无效值
  int16_t vis_tgt_y_ = 0; ///< 视觉瞄准目标的 y 坐标，单位为像素，负数为无效值

  // var for hurt warning
  bool is_armor_hit_ = false; ///< 装甲是否被击中
  uint8_t
      armor_id_hit_ = 4,
      last_armor_id_hit_ =
          4; ///< 被击中装甲的 ID,初始设置>3,防止小陀螺模式首次受击无法触发提示
  float last_armor_angle_hit_ = 0.0f;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
} // namespace robot

#endif /* CHASSIS_ROBOT_MODULE_UI_DRAWER_HPP_ */
