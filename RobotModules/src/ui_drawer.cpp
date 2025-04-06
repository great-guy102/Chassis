/**
 *******************************************************************************
 * @file      :ui_drawer.cpp
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
#include <string>

#include "rfr_pkg/rfr_pkg_0x0301_inter_graphics.hpp"
#include "ui_drawer.hpp"

// typedef hello_world::referee hello_world::referee ;
/* Private macro -------------------------------------------------------------*/

namespace robot {
/* Private constants ---------------------------------------------------------*/

#pragma region names of graphics

const hello_world::referee::GraphicLayer kStaticUiLayer =
    hello_world::referee::GraphicLayer::k0;
const hello_world::referee::GraphicLayer kDynamicUiLayer =
    hello_world::referee::GraphicLayer::k1;

const hello_world::referee::Graphic::Color kUiNormalColor =
    hello_world::referee::Graphic::Color::kGreen;
const hello_world::referee::Graphic::Color kUiWarningColor =
    hello_world::referee::Graphic::Color::kOrange;
const hello_world::referee::Graphic::Color kUiErrorColor =
    hello_world::referee::Graphic::Color::kPurple;

const hello_world::referee::Graphic::Color kUiModuleStateTitleColor =
    hello_world::referee::Graphic::Color::kOrange;
const hello_world::referee::Graphic::Color kUiPassLineColor =
    hello_world::referee::Graphic::Color::kOrange;
const hello_world::referee::Graphic::Color kUiVisTgtColor =
    hello_world::referee::Graphic::Color::kGreen;
const hello_world::referee::Graphic::Color kUiVisBoxColor =
    hello_world::referee::Graphic::Color::kWhite;

const hello_world::referee::Pixel kUiModuleStateFontSize = 16;
const hello_world::referee::Pixel kUiModuleStateLineWidth = 3;
const int16_t kUiModuleStateAreaYDelta = -35;

const uint16_t kUiScreenMiddleX = 1920 / 2; // 操作界面中心位置
const uint16_t kUiScreenMiddleY = 1080 / 2; // 操作界面中心位置

const uint16_t kUiModuleStateAreaX1 = 15;
const uint16_t kUiModuleStateAreaX2_1 = 140;
const uint16_t kUiModuleStateAreaX2_2 =
    kUiModuleStateAreaX2_1 - kUiModuleStateFontSize;
const uint16_t kUiModuleStateAreaX2_3 = kUiModuleStateAreaX2_1;
const uint16_t kUiModuleStateAreaX3 = kUiScreenMiddleX;
const uint16_t kUiModuleStateAreaY1 = 890;
const uint16_t kUiModuleStateAreaY2 = 700;

// 各模块状态 左上角
// chassis
const uint8_t kUiNameChassisWorkStateTitle[3] = {0x00, 0x00,
                                                 0x01}; ///< 底盘工作状态标题
const uint8_t kUiNameChassisWorkStateContent[3] = {0x00, 0x00,
                                                   0x02}; ///< 底盘工作状态内容
const uint8_t kUiNameChassisDirHead[3] = {0x00, 0x00,
                                          0x03}; ///< 底盘朝向示意（底盘头部）
const uint8_t kUiNameChassisDirTail[3] = {0x00, 0x00,
                                          0x04}; ///< 底盘朝向示意（底盘尾部）

const uint16_t kPixelCenterXCapBox = kUiScreenMiddleX; // 超电位置
const uint16_t kPixelCenterYCapBox = 120;
const uint16_t kPixelCapBoxWidth = 400; // 超电能量余量外框
const uint16_t kPixelCapBoxHeight = 16;
const uint8_t kUiNameChassisCapBox[3] = {0x00, 0x00,
                                         0x05}; ///< 超级电容能量余量外框
const uint8_t kUiNameChassisCapPercent[3] = {
    0x00, 0x00, 0x06}; ///< 超级电容能量余量百分比示意
const uint8_t kUiNameChassisCapPercentNum[3] = {
    0x00, 0x00, 0x07}; ///< 超级电容能量余量百分比数字

// 行车线
const uint8_t kuiNameChassisPassLineLeft[3] = {0x00, 0x00,
                                               0x08}; ///< 底盘通行线左侧
const uint8_t kuiNameChassisPassLineRight[3] = {0x00, 0x00,
                                                0x09}; ///< 底盘通行线右侧

// gimbal
const uint16_t kPixelCenterXVisionBox = 1704.5 / 2; // TODO： 云台视觉状态位置
const uint16_t kPixelCenterYVisionBox = 1198.3 / 2;
const uint16_t kPixelVisionBoxWidth = 592.5; // 状态外框 云台视觉
const uint16_t kPixelVisionBoxHeight = 315.3;

const uint8_t kUiNameGimbalWorkStateTitle[3] = {0x00, 0x00,
                                                0x40}; ///< 云台工作状态标题
const uint8_t kUiNameGimbalWorkStateContent[3] = {0x00, 0x00,
                                                  0x41}; ///< 云台工作状态内容

const uint8_t kUiNameGimbalPitchTitle[3] = {0x00, 0x00,
                                            0x42}; ///< 云台俯仰角度标题
const uint8_t kUiNameGimbalPitchFdb[3] = {0x00, 0x00,
                                          0x43}; ///< 云台俯仰角度反馈

const uint8_t kUiNameGimbalYawTitle[3] = {0x00, 0x00,
                                          0x44};           ///< 云台偏航角度标题
const uint8_t kUiNameGimbalYawFdb[3] = {0x00, 0x00, 0x45}; ///< 云台偏航角度反馈
const uint8_t kUiNamePassSafe[3] = {0x00, 0x00, 0x0A};     ///< 云台过洞安全提示

// shooter
const uint8_t kUiNameShooterWorkStateTitle[3] = {
    0x00, 0x00, 0x80}; ///< 发射机构工作状态标题
const uint8_t kUiNameShooterWorkStateContent[3] = {
    0x00, 0x00, 0x81}; ///< 发射机构工作状态内容

const uint8_t kUiNameShooterHeat[3] = {0x00, 0x00, 0x82}; ///< 发射机构热量

// 瞄准线 正中
// const uint16_t kUiAimLineX = 987;
// const uint16_t kUiAimLineH5mY = 700;
// const uint16_t kUiAimErrH5m = 20;
// const uint16_t kUiAimLineH8mY = 660;
// const uint16_t kUiAimErrH8m = 15;
// const uint16_t kUiAimLineH10mY = 620;
// const uint16_t kUiAimErrH10m = 10;
// const uint16_t kUiAimLineH15mY = 580;
// const uint16_t kUiAimErrH15m = 5;

// const uint8_t kUiNameAimLineV[3] = {0x00, 0x00, 0x8B};     ///< 瞄准线垂直
// const uint8_t kUiNameAimLineH5m[3] = {0x00, 0x00, 0x8C};   ///< 瞄准线水平 5m
// const uint8_t kUiNameAimLineH8m[3] = {0x00, 0x00, 0x8D};   ///< 瞄准线水平 8m
// const uint8_t kuiNameAimLineH10m[3] = {0x00, 0x00, 0x8E};  ///< 瞄准线水平
// 10m const uint8_t kuiNameAimLineH15m[3] = {0x00, 0x00, 0x8F};  ///<瞄准线水平
// 15m mini gimbal

// vision
const uint8_t kUiNameVisionBox[3] = {0x00, 0x00, 0xE0}; ///< 视觉相机视场框
const uint8_t kUiNameVisionTgt[3] = {0x00, 0x00, 0xE1}; ///< 视觉相机目标框

// hurt
const uint8_t kUiNameHitWarning[3] = {0x00, 0x00, 0xE2}; ///< 装甲被击中提示

// 安全过洞参数
const float ksafepitchmin = 0.0; // TODO：待标定
const float ksafepitchmax = 0.1; // TODO
#pragma endregion names of graphics

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

bool UiDrawer::encode(uint8_t *data_ptr, size_t &data_len) {
  bool res = false;
  bool is_all_added = (n_added_ == kNumAllPkgs);
  hello_world::referee::GraphicOperation opt =
      hello_world::referee::GraphicOperation::kAdd;
  if (is_all_added) {
    opt = hello_world::referee::GraphicOperation::kModify;
  }
  if (ui_idx_ >= kNumAllPkgs) {
    if (is_all_added) {
      ui_idx_ = kSuiPkgNum;
    } else {
      ui_idx_ = 0;
    }
  }

  if (ui_idx_ < kSuiPkgNum) {
    res = encodeStaticUi(data_ptr, data_len, opt, StaticUiIdx(ui_idx_));
  } else if (ui_idx_ < kNumAllPkgs) {
    res = encodeDynamicUi(data_ptr, data_len, opt,
                          DynamicUiIdx(ui_idx_ - kSuiPkgNum));
  };

  if (res) {
    ui_idx_++;
  }

  if (res && (is_all_added == false)) {
    n_added_++;
  }
  return res;
};

bool UiDrawer::encodeStaticUi(uint8_t *data_ptr, size_t &data_len,
                              GraphicOperation opt, StaticUiIdx idx) {
  switch (idx) {
  case kSuiDelAll:
    return encodeDelAll(data_ptr, data_len);
    break;
  case kSuiChassisTitle:
    return encodeChassisWorkStateTitle(data_ptr, data_len, opt);
    break;
  case kSuiGimbalTitle:
    return encodeGimbalWorkStateTitle(data_ptr, data_len, opt);
    break;
  case kSuiShooterTitle:
    return encodeShooterWorkStateTitle(data_ptr, data_len, opt);
    break;
  case kSuiPkgGroup1:
    //   return encodeStaticPkgGroup1(data_ptr, data_len, opt);
    break;
  case kSuiPkgGroup2:
    return encodeStaticPkgGroup2(data_ptr, data_len, opt);
    break;

  default:
    break;
  }
  return true;
};
bool UiDrawer::encodeDynamicUi(uint8_t *data_ptr, size_t &data_len,
                               GraphicOperation opt, DynamicUiIdx idx) {
  bool res = true;
  switch (idx) {
  case kDuiChassisContent:
    if (opt == hello_world::referee::GraphicOperation::kModify) {
      if (last_chassis_work_state_ == chassis_work_state_ &&
          last_chassis_working_mode_ == chassis_working_mode_ &&
          last_chassis_ctrl_mode_ == chassis_ctrl_mode_ &&
          last_chassis_manual_ctrl_src_ == chassis_manual_ctrl_src_) {
        return true;
      }
    } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
    } else {
      return true;
    }

    res = encodeChassisWorkStateContent(data_ptr, data_len, opt);
    if (res == true) {
      last_chassis_work_state_ = chassis_work_state_;
      last_chassis_working_mode_ = chassis_working_mode_;
      last_chassis_ctrl_mode_ = chassis_ctrl_mode_;
      last_chassis_manual_ctrl_src_ = chassis_manual_ctrl_src_;
    }

    break;

  case kDuiGimbalContent:
    if (opt == hello_world::referee::GraphicOperation::kModify) {
      if (last_gimbal_work_state_ == gimbal_work_state_ &&
          last_gimbal_working_mode_ == gimbal_working_mode_ &&
          last_gimbal_ctrl_mode_ == gimbal_ctrl_mode_ &&
          last_gimbal_manual_ctrl_src_ == gimbal_manual_ctrl_src_) {
        return true;
      }
    } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
    } else {
      return true;
    }

    res = encodeGimbalWorkStateContent(data_ptr, data_len, opt);
    if (res == true) {
      last_gimbal_work_state_ = gimbal_work_state_;
      last_gimbal_working_mode_ = gimbal_working_mode_;
      last_gimbal_ctrl_mode_ = gimbal_ctrl_mode_;
      last_gimbal_manual_ctrl_src_ = gimbal_manual_ctrl_src_;
    }

    break;
  case kDuiShooterContent:
    if (opt == hello_world::referee::GraphicOperation::kModify) {
      if (last_shooter_work_state_ == shooter_work_state_ &&
          last_shooter_working_mode_ == shooter_working_mode_ &&
          last_shooter_ctrl_mode_ == shooter_ctrl_mode_ &&
          last_shooter_manual_ctrl_src_ == shooter_manual_ctrl_src_) {
        return true;
      }
    } else if (opt == hello_world::referee::GraphicOperation::kAdd) {
    } else {
      return true;
    }
    res = encodeShooterWorkStateContent(data_ptr, data_len, opt);
    if (res == true) {
      last_shooter_work_state_ = shooter_work_state_;
      last_shooter_working_mode_ = shooter_working_mode_;
      last_shooter_ctrl_mode_ = shooter_ctrl_mode_;
      last_shooter_manual_ctrl_src_ = shooter_manual_ctrl_src_;
    }

    break;
  case kDuiPkgGroup1:
    res = encodeDynaUiPkgGroup1(data_ptr, data_len, opt);
    break;
  case kDuiPkgGroup2:
    res = encodeDynaUiPkgGroup2(data_ptr, data_len, opt);
    break;
  case kDuiPkgGroup3:
    res = encodeDynaUiPkgGroup3(data_ptr, data_len, opt);
    break;
  default:
    break;
  }

  return res;
};

bool UiDrawer::encodeDelAll(uint8_t *data_ptr, size_t &data_len) {
  hello_world::referee::InterGraphicDeletePackage pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setDeleteOperation(hello_world::referee::DeleteOperation::kAll);
  return encoder_.encodeFrame(&pkg, data_ptr, &data_len);
};

#pragma region
#pragma region UI 组
// bool UiDrawer::encodeStaticPkgGroup1(uint8_t* data_ptr, size_t& data_len,
// GraphicOperation opt)
// {
//   hello_world::referee::StraightLine g_aim_line_v, g_aim_line_h5m,
//   g_aim_line_h8m, g_aim_line_h10m, g_aim_line_h15m;
//   genAimLineV(g_aim_line_v);
//   g_aim_line_v.setOperation(opt);
//   genAimLineH5m(g_aim_line_h5m);
//   g_aim_line_h5m.setOperation(opt);
//   genAimLineH8m(g_aim_line_h8m);
//   g_aim_line_h8m.setOperation(opt);
//   genAimLineH10m(g_aim_line_h10m);
//   g_aim_line_h10m.setOperation(opt);
//   genAimLineH15m(g_aim_line_h15m);
//   g_aim_line_h15m.setOperation(opt);

//   hello_world::referee::InterGraphic5Package pkg;
//   pkg.setSenderId(static_cast<uint16_t>(sender_id_));
//   pkg.setStraightLineAt(g_aim_line_v, 0);
//   pkg.setStraightLineAt(g_aim_line_h5m, 1);
//   pkg.setStraightLineAt(g_aim_line_h8m, 2);
//   pkg.setStraightLineAt(g_aim_line_h10m, 3);
//   pkg.setStraightLineAt(g_aim_line_h15m, 4);
//   return encodePkg(data_ptr, data_len, opt, pkg);
// };
bool UiDrawer::encodeStaticPkgGroup2(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt) {
  // 新增视觉框
  hello_world::referee::Rectangle g_vision_box;
  genVisionbox(g_vision_box);
  g_vision_box.setOperation(opt);

  hello_world::referee::InterGraphic1Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setRectangle(g_vision_box);
  return encodePkg(data_ptr, data_len, opt, pkg);
};

bool UiDrawer::encodeDynaUiPkgGroup1(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt) {
  hello_world::referee::Arc g_chassis_head, g_chassis_tail;

  genChassisDir(g_chassis_head, g_chassis_tail);
  g_chassis_head.setOperation(opt);
  g_chassis_tail.setOperation(opt);

  hello_world::referee::InterGraphic2Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setArcAt(g_chassis_head, 0);
  pkg.setArcAt(g_chassis_tail, 1);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
bool UiDrawer::encodeDynaUiPkgGroup2(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt) {
  hello_world::referee::Rectangle g_cap_pwr_percent_rect;
  hello_world::referee::FloatingNumber g_cap_pwr_percent_num;

  genCapPwrPercent(g_cap_pwr_percent_rect, g_cap_pwr_percent_num);
  g_cap_pwr_percent_rect.setOperation(opt);
  g_cap_pwr_percent_num.setOperation(opt);

  hello_world::referee::StraightLine g_pass_line_left, g_pass_line_right;
  genChassisPassLineLeft(g_pass_line_left);
  g_pass_line_left.setOperation(opt);
  genChassisPassLineRight(g_pass_line_right);
  g_pass_line_right.setOperation(opt);

  // 显示过洞角度是否安全
  hello_world::referee::Circle g_pass_hole;
  bool is_safe = false;
  if (gimbal_joint_angle_pitch_fdb_ > ksafepitchmin &&
      gimbal_joint_angle_pitch_fdb_ < ksafepitchmax) {
    is_safe = true;
  } else {
    is_safe = false;
  }
  genPassSafe(g_pass_hole, is_safe);
  g_pass_hole.setOperation(opt);

  hello_world::referee::InterGraphic5Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setRectangleAt(g_cap_pwr_percent_rect, 0);
  pkg.setFloatingNumberAt(g_cap_pwr_percent_num, 1);
  pkg.setStraightLineAt(g_pass_line_left, 2);
  pkg.setStraightLineAt(g_pass_line_right, 3);
  pkg.setCircleAt(g_pass_hole, 4);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
bool UiDrawer::encodeDynaUiPkgGroup3(uint8_t *data_ptr, size_t &data_len,
                                     GraphicOperation opt) {
  hello_world::referee::Arc g_heat, g_armor_hit;
  hello_world::referee::Circle g_vision;
  hello_world::referee::Integer g_balence_number;

  genShooterHeat(g_heat);
  g_heat.setOperation(opt);

  genVisTgt(g_vision);
  g_vision.setOperation(opt);

  genArmorHit(g_armor_hit);
  g_armor_hit.setOperation(opt);

  hello_world::referee::InterGraphic5Package pkg;
  pkg.setSenderId(static_cast<uint16_t>(sender_id_));
  pkg.setArcAt(g_heat, 0);
  pkg.setCircleAt(g_vision, 1);
  pkg.setArcAt(g_heat, 2);
  pkg.setCircleAt(g_vision, 3);
  pkg.setArcAt(g_armor_hit, 4);
  return encodePkg(data_ptr, data_len, opt, pkg);
};
#pragma endregion

#pragma region 底盘相关 UI

/**
 * @brief 编码左上方 UI 字符串 `Chassis:`
 */
bool UiDrawer::encodeChassisWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                           GraphicOperation opt) {
  std::string str = "Chassis:";
  hello_world::referee::String options = hello_world::referee::String(
      kUiNameChassisWorkStateTitle, opt, kStaticUiLayer,
      kUiModuleStateTitleColor, kUiModuleStateAreaX1, kUiModuleStateAreaY1,
      kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, options, str);
};

/**
 * @brief 根据底盘工作状态编码左上方 UI 字符串(`Chassis:` 之后的内容)
 */
bool UiDrawer::encodeChassisWorkStateContent(uint8_t *data_ptr,
                                             size_t &data_len,
                                             UiDrawer::GraphicOperation opt) {
  std::string str = "Unknown";
  hello_world::referee::Graphic::Color color = kUiNormalColor;

  if (chassis_work_state_ != PwrState::kWorking) {
    color = kUiWarningColor;
    str = PwrStateToStr(chassis_work_state_);
  } else {
    str = Chassis::WorkingModeToStr(chassis_working_mode_) + "-" +
          CtrlModeSrcToStr(chassis_ctrl_mode_, chassis_manual_ctrl_src_);
  }

  hello_world::referee::String options = hello_world::referee::String(
      kUiNameChassisWorkStateContent, opt, kDynamicUiLayer, color,
      kUiModuleStateAreaX2_1, kUiModuleStateAreaY1, kUiModuleStateFontSize,
      str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, options, str);
};

void UiDrawer::genChassisDir(hello_world::referee::Arc &g_head,
                             hello_world::referee::Arc &g_tail) {
  float armor_angle_head = -theta_i2r_ * 180 / M_PI;
  float arc_angle_head_start = armor_angle_head - 40,
        arc_angle_head_end = armor_angle_head + 40;
  arc_angle_head_start =
      hello_world::NormPeriodData(0, 360, arc_angle_head_start);
  arc_angle_head_end = hello_world::NormPeriodData(0, 360, arc_angle_head_end);

  float arc_angle_tail_start = arc_angle_head_start + 180,
        arc_angle_tail_end = arc_angle_head_end + 180;
  arc_angle_tail_start =
      hello_world::NormPeriodData(0, 360, arc_angle_tail_start);
  arc_angle_tail_end = hello_world::NormPeriodData(0, 360, arc_angle_tail_end);

  float radius = 50; // 灯条所处圆的半径
  g_head.setColor(hello_world::referee::Graphic::Color::kYellow);
  g_head.setAng(arc_angle_head_start, arc_angle_head_end);
  g_head.setCenterPos(kUiScreenMiddleX, kUiScreenMiddleY);
  g_head.setRadius(radius, radius);
  g_head.setName(kUiNameChassisDirHead);
  g_head.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);

  g_tail.setColor(hello_world::referee::Graphic::Color::kCyan);
  g_tail.setAng(arc_angle_tail_start, arc_angle_tail_end);
  g_tail.setCenterPos(kUiScreenMiddleX, kUiScreenMiddleY);
  g_tail.setRadius(radius, radius);
  g_tail.setName(kUiNameChassisDirTail);
  g_tail.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);
};

void UiDrawer::genCapPwrPercent(hello_world::referee::Rectangle &g_rect,
                                hello_world::referee::FloatingNumber &g_num) {
  uint16_t start_x = kPixelCenterXCapBox - kPixelCapBoxWidth / 2;
  float percent = hello_world::Bound(cap_pwr_percent_ / 100.0f, 0, 1);
  uint16_t end_x = start_x + kPixelCapBoxWidth * percent;

  hello_world::referee::Graphic::Color color;

  if (percent > 0.75) {
    color = kUiNormalColor;
  } else if (percent > 0.3) {
    color = kUiWarningColor;
  } else {
    color = kUiErrorColor;
  }

  g_rect.setName(kUiNameChassisCapPercent);
  g_rect.setStartPos(start_x, kPixelCenterYCapBox - kPixelCapBoxHeight / 2);
  g_rect.setEndPos(end_x, kPixelCenterYCapBox + kPixelCapBoxHeight / 2);
  g_rect.setColor(color);
  g_rect.setLayer(kDynamicUiLayer);
  g_rect.setLineWidth(kPixelCapBoxHeight * 2);

  g_num.setName(kUiNameChassisCapPercentNum);
  g_num.setDisplayValue(percent * 100);
  g_num.setStartPos(start_x - 106,
                    kPixelCenterYCapBox + kPixelCapBoxHeight / 2);
  g_num.setColor(color);
  g_num.setLayer(kDynamicUiLayer);
  g_num.setFontSize(24); // 调整超电剩余电量的数字大小
  g_num.setLineWidth(kUiModuleStateLineWidth);
};

void UiDrawer::genChassisPassLineLeft(hello_world::referee::StraightLine &g) {
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  g.setName(kuiNameChassisPassLineLeft);
  end_posX = gimbal_joint_angle_pitch_fdb_ * (-81.75) + 814.7;
  start_posX = gimbal_joint_angle_pitch_fdb_ * 660.4 + 577.15;
  end_posY = gimbal_joint_angle_pitch_fdb_ * (-927.28) + 334.39;
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(kUiPassLineColor);
  g.setLineWidth(3);
};
void UiDrawer::genChassisPassLineRight(hello_world::referee::StraightLine &g) {
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  start_posX = gimbal_joint_angle_pitch_fdb_ * (-801.84) + 1277.1;
  end_posX = gimbal_joint_angle_pitch_fdb_ * 96.48 + 1077;
  end_posY = gimbal_joint_angle_pitch_fdb_ * (-927.28) + 334.39;
  g.setName(kuiNameChassisPassLineRight);
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(kUiPassLineColor);
  g.setLineWidth(3);
};
#pragma endregion
#pragma region 云台相关 UI
/**
 * @brief 编码左上方 UI 字符串 `Gimbal:`
 */
bool UiDrawer::encodeGimbalWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                          UiDrawer::GraphicOperation opt) {
  std::string str = "Gimbal:";

  hello_world::referee::String options = hello_world::referee::String(
      kUiNameGimbalWorkStateTitle, opt, kStaticUiLayer,
      kUiModuleStateTitleColor, kUiModuleStateAreaX1,
      kUiModuleStateAreaY1 + kUiModuleStateAreaYDelta, kUiModuleStateFontSize,
      str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, options, str);
};
/**
 * @brief 根据云台工作状态编码左上方 UI 字符串(`Gimbal:` 之后的内容)
 */
bool UiDrawer::encodeGimbalWorkStateContent(uint8_t *data_ptr, size_t &data_len,
                                            UiDrawer::GraphicOperation opt) {
  std::string str = "Unknown";
  hello_world::referee::Graphic::Color color = kUiNormalColor;
  if (gimbal_work_state_ != PwrState::kWorking) {
    color = kUiWarningColor;
    str = PwrStateToStr(gimbal_work_state_);
  } else {
    str = Gimbal::WorkingModeToStr(gimbal_working_mode_) + "-" +
          CtrlModeSrcToStr(gimbal_ctrl_mode_, gimbal_manual_ctrl_src_);
  }

  hello_world::referee::String options = hello_world::referee::String(
      kUiNameGimbalWorkStateContent, opt, kDynamicUiLayer, color,
      kUiModuleStateAreaX2_2, kUiModuleStateAreaY1 + kUiModuleStateAreaYDelta,
      kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, options, str);
};

void UiDrawer::genPassSafe(hello_world::referee::Circle &g, bool is_safe) {
  g.setName(kUiNamePassSafe);
  g.setCenterPos(kUiModuleStateAreaX3,
                 kUiModuleStateAreaY1 +
                     kUiModuleStateAreaYDelta); // TODO：确认该指示显示位置
  g.setRadius(25);
  g.setColor(is_safe ? kUiNormalColor : kUiErrorColor);
  g.setLayer(kDynamicUiLayer);
  g.setLineWidth(3);
};
#pragma endregion

#pragma region 发射机构相关 UI

//  * @brief 编码左上方 UI 字符串 `Shooter:`
//  */
bool UiDrawer::encodeShooterWorkStateTitle(uint8_t *data_ptr, size_t &data_len,
                                           UiDrawer::GraphicOperation opt) {
  std::string str = "Shooter:";

  hello_world::referee::String options = hello_world::referee::String(
      kUiNameShooterWorkStateTitle, opt, kStaticUiLayer,
      kUiModuleStateTitleColor, kUiModuleStateAreaX1,
      kUiModuleStateAreaY1 + 2 * kUiModuleStateAreaYDelta,
      kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);
  return encodeString(data_ptr, data_len, opt, options, str);
};
/**
 * @brief 根据射门工作状态编码左上方 UI 字符串(`Shooter:` 之后的内容)
 */
bool UiDrawer::encodeShooterWorkStateContent(uint8_t *data_ptr,
                                             size_t &data_len,
                                             UiDrawer::GraphicOperation opt) {
  std::string str = "Unknown";
  hello_world::referee::Graphic::Color color = kUiNormalColor;
  if (shooter_work_state_ != PwrState::kWorking) {
    str = PwrStateToStr(shooter_work_state_);
    color = kUiWarningColor;
  } else {
    if (shooter_stuck_flag_) {
      color = kUiErrorColor;
      switch (feed_stuck_status_) {
      case 0:
        str = "Fric Stuck";
        break;
      case 1:
        str = "Forward Stuck";
        break;
      case 2:
        str = "Backward Stuck";
        break;
      default:
        str = "Unknown";
        break;
      }
    } else {
      str = Shooter::WorkingModeToStr(shooter_working_mode_) + "-" +
            CtrlModeSrcToStr(shooter_ctrl_mode_, shooter_manual_ctrl_src_);
    }
  }

  hello_world::referee::String options = hello_world::referee::String(
      kUiNameShooterWorkStateContent, opt, kDynamicUiLayer, color,
      kUiModuleStateAreaX2_3,
      kUiModuleStateAreaY1 + 2 * kUiModuleStateAreaYDelta,
      kUiModuleStateFontSize, str.length(), kUiModuleStateLineWidth);

  return encodeString(data_ptr, data_len, opt, options, str);
};

void UiDrawer::genShooterHeat(hello_world::referee::Arc &g) {
  float percent = heat_limit_ > 0 ? heat_ / heat_limit_ : 0.0f;
  percent = hello_world::Bound(percent, 0.0f, 1.0f);

  hello_world::referee::Graphic::Color color;
  if (percent > 0.8) {
    color = kUiNormalColor;
  } else if (percent > 0.6) {
    color = kUiWarningColor;
  } else if (percent > 0.3) {
    color = kUiErrorColor;
  }

  g.setName(kUiNameShooterHeat);
  g.setCenterPos(kUiScreenMiddleX, kUiScreenMiddleY);
  g.setRadius(100, 100);
  g.setAng(360 - 360 * percent, 0);
  g.setColor(color);
  g.setLineWidth(percent == 0 ? 0 : 2);
  g.setLayer(kDynamicUiLayer);
};
#pragma endregion

#pragma region 视觉 UI

void UiDrawer::genVisTgt(hello_world::referee::Circle &g) {
  if (is_vision_valid_) {
    g.setLineWidth(3);
  } else {
    g.setLineWidth(0);
  }

  g.setName(kUiNameVisionTgt);
  g.setCenterPos(960 + vis_tgt_x_ - 100, 540 - vis_tgt_y_);
  g.setRadius(30);
  g.setColor(kUiNormalColor);
  g.setLayer(kDynamicUiLayer);
};

void UiDrawer::genVisionbox(hello_world::referee::Rectangle &g_rect) {
  // uint16_t start_x = kPixelCenterXVisionBox - kPixelVisionBoxWidth / 2;
  // uint16_t end_x = start_x + kPixelVisionBoxWidth;

  // g_rect.setStartPos(start_x, kPixelCenterYVisionBox - kPixelVisionBoxHeight
  // / 2);
  // g_rect.setEndPos(end_x, kPixelCenterYVisionBox + kPixelVisionBoxHeight /
  // 2);
  g_rect.setName(kUiNameVisionBox);
  g_rect.setStartPos(655, 250);
  g_rect.setEndPos(1269, 651);
  g_rect.setColor(kUiVisBoxColor);
  g_rect.setLineWidth(2);
  g_rect.setLayer(kStaticUiLayer);
};

#pragma endregion

#pragma region 受击反馈
void UiDrawer::genArmorHit(hello_world::referee::Arc &g_hit) {
  // 装甲板被击中时的警告角度,小陀螺模式因数据链路延迟等原因，会导致提示方向有一定误差
  // 故将范围角度进行一定预判性偏置并适当扩大提示范围,具体偏置角度待测试
  // 偏置在底盘顺时针小陀螺时应为正值，逆时针反之
  const float kHitArcAngleRangeGyro = 90.0f;
  const float kHitArcAngleBiasGyro = 10.0f;
  const float kHitArcAngleRangeFollow = 80.0f;

  if (is_armor_hit_) {
    g_hit.setLineWidth(6);
  } else {
    g_hit.setLineWidth(0);
  }

  // 跟随模式前向对应的装甲板应设置为1号，其余装甲板序号由裁判系统要求，逆时针递增
  float armor_angle_hit = 0.0f, arc_angle_hit_start = 0.0f,
        arc_angle_hit_end = 0.0f;
  if (chassis_working_mode_ == Chassis::WorkingMode::Gyro) {
    if (armor_id_hit_ == last_armor_id_hit_) {
      armor_angle_hit = last_armor_angle_hit_;
    }
    arc_angle_hit_start =
        armor_angle_hit - kHitArcAngleRangeGyro / 2.0f + kHitArcAngleBiasGyro,
    arc_angle_hit_end =
        armor_angle_hit + kHitArcAngleRangeGyro / 2.0f + kHitArcAngleBiasGyro;
  } else {
    armor_angle_hit = -(theta_i2r_ * 180.0f / M_PI + 90.0f * armor_id_hit_);
    arc_angle_hit_start = armor_angle_hit - kHitArcAngleRangeFollow / 2.0f;
    arc_angle_hit_end = armor_angle_hit + kHitArcAngleRangeFollow / 2.0f;
  }

  arc_angle_hit_start =
      hello_world::NormPeriodData(0.0f, 360.0f, arc_angle_hit_start);
  arc_angle_hit_end =
      hello_world::NormPeriodData(0.0f, 360.0f, arc_angle_hit_end);

  float radius = 200.0f; // 灯条所处圆的半径
  g_hit.setColor(hello_world::referee::Graphic::Color::kPurple);
  g_hit.setAng(arc_angle_hit_start, arc_angle_hit_end);
  g_hit.setCenterPos(kUiScreenMiddleX,
                     kUiScreenMiddleY); // 灯条中心位置
  g_hit.setRadius(radius, radius);
  g_hit.setName(kUiNameHitWarning);
  g_hit.setLayer(kDynamicUiLayer);
  last_armor_id_hit_ = armor_id_hit_;
  last_armor_angle_hit_ = armor_angle_hit;
};

#pragma endregion
/* Private function definitions ----------------------------------------------*/

} // namespace robot