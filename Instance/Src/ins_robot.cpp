/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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
#include "ins_all.hpp"
/* Private constants ---------------------------------------------------------*/
const robot::Chassis::Config kChassisConfig = {
    .normal_trans_vel = 5.0f,    ///< 正常平移速度
    .gyro_rot_spd_move = 7.0f,   ///< 小陀螺旋转速度
    .gyro_rot_spd_stand = 12.0f, ///< 静止时小陀螺旋转速度
    .yaw_sensitivity = 2 * PI,   ///< YAW 轴灵敏度(单位：rad/s)
    .max_trans_vel = 5.0f,       ///< 最大平移速度
    .max_rot_spd = 14.0f,        ///< 最大旋转速度
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static bool is_robot_inited = false;
static bool is_chassis_inited = false;

robot::Robot unique_robot = robot::Robot();
robot::Chassis unique_chassis = robot::Chassis(kChassisConfig);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Chassis *GetChassis() {
  if (!is_chassis_inited) {
    // * 1. 无通信功能的组件指针
    // * - 底盘逆解
    unique_chassis.registerIkSolver(GetChassisIkSolver());
    // * - 斜坡
    unique_chassis.registerRampCmdVx(GetRampCmdVx());
    unique_chassis.registerRampCmdVy(GetRampCmdVy());
    unique_chassis.registerRampCmdV(GetRampCmdV());
    // * - pid
    // 轮组 pid
    unique_chassis.registerWheelPid(GetPidMotorWheelLeftFront(),
                                    robot::Chassis::kWheelPidIdxLeftFront);
    unique_chassis.registerWheelPid(GetPidMotorWheelLeftRear(),
                                    robot::Chassis::kWheelPidIdxLeftRear);
    unique_chassis.registerWheelPid(GetPidMotorWheelRightRear(),
                                    robot::Chassis::kWheelPidIdxRightRear);
    unique_chassis.registerWheelPid(GetPidMotorWheelRightFront(),
                                    robot::Chassis::kWheelPidIdxRightFront);

    unique_chassis.registerSteerPid(GetPidMotorSteerLeftFront(),
                                    robot::Chassis::kSteerPidIdxLeftFront);
    unique_chassis.registerSteerPid(GetPidMotorSteerLeftRear(),
                                    robot::Chassis::kSteerPidIdxLeftRear);
    unique_chassis.registerSteerPid(GetPidMotorSteerRightRear(),
                                    robot::Chassis::kSteerPidIdxRightRear);
    unique_chassis.registerSteerPid(GetPidMotorSteerRightFront(),
                                    robot::Chassis::kSteerPidIdxRightFront);
    // 随动速度
    unique_chassis.registerFollowOmegaPid(GetPidFollowOmega());
    // * - 功率限制
    unique_chassis.registerPwrLimiter(GetPwrLimiter());

    // * 2. 只接收数据的组件指针
    // 云台和底盘通信
    unique_chassis.registerGimbalChassisComm(GetGimbalChassisComm());
    // YAW 轴电机
    unique_chassis.registerYawMotor(GetMotorYaw());
    // * 3. 接收、发送数据的组件指针
    // 超级电容
    unique_chassis.registerCap(GetCap());
    // 轮组电机
    unique_chassis.registerWheelMotor(GetMotorWheelLeftFront(),
                                      robot::Chassis::kWheelMotorIdxLeftFront);
    unique_chassis.registerWheelMotor(GetMotorWheelLeftRear(),
                                      robot::Chassis::kWheelMotorIdxLeftRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightRear(),
                                      robot::Chassis::kWheelMotorIdxRightRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightFront(),
                                      robot::Chassis::kWheelMotorIdxRightFront);
    // 舵机
    unique_chassis.registerSteerMotor(GetMotorSteerLeftFront(),
                                      robot::Chassis::kSteerMotorIdxLeftFront);
    unique_chassis.registerSteerMotor(GetMotorSteerLeftRear(),
                                      robot::Chassis::kSteerMotorIdxLeftRear);
    unique_chassis.registerSteerMotor(GetMotorSteerRightRear(),
                                      robot::Chassis::kSteerMotorIdxRightRear);
    unique_chassis.registerSteerMotor(GetMotorSteerRightFront(),
                                      robot::Chassis::kSteerMotorIdxRightFront);
    is_chassis_inited = true;
  }
  return &unique_chassis;
};

robot::Robot *GetRobot() {
  if (!is_robot_inited) {
    // main 组件指针注册
    // 主要模块状态机组件指针
    unique_robot.registerChassis(GetChassis());

    // 无通信功能的组件指针
    unique_robot.registerBuzzer(GetBuzzer());
    unique_robot.registerImu(GetImu());

    // 只接收数据的组件指针
    // 只发送数据的组件指针
    // CAN1
    unique_robot.registerCap(GetCap());
    unique_robot.registerMotorSteers(GetMotorSteerLeftFront(),
                                     robot::Robot::kSteerMotorIdxLeftFront);
    unique_robot.registerMotorSteers(GetMotorSteerLeftRear(),
                                     robot::Robot::kSteerMotorIdxLeftRear);
    unique_robot.registerMotorSteers(GetMotorSteerRightRear(),
                                     robot::Robot::kSteerMotorIdxRightRear);
    unique_robot.registerMotorSteers(GetMotorSteerRightFront(),
                                     robot::Robot::kSteerMotorIdxRightFront);
    // CAN2
    unique_robot.registerMotorWheels(GetMotorWheelLeftFront(),
                                     robot::Robot::kWheelMotorIdxLeftFront);
    unique_robot.registerMotorWheels(GetMotorWheelLeftRear(),
                                     robot::Robot::kWheelMotorIdxLeftRear);
    unique_robot.registerMotorWheels(GetMotorWheelRightRear(),
                                     robot::Robot::kWheelMotorIdxRightRear);
    unique_robot.registerMotorWheels(GetMotorWheelRightFront(),
                                     robot::Robot::kWheelMotorIdxRightFront);
    // 收发数据的组件指针
    // CAN2
    unique_robot.registerGimbalChassisComm(GetGimbalChassisComm());
    // USART6
    unique_robot.registerReferee(GetReferee());

    unique_robot.registerPerformancePkg(GetRobotPerformancePackage());
    unique_robot.registerPowerHeatPkg(GetRobotPowerHeatPackage());
    unique_robot.registerShooterPkg(GetRobotShooterPackage());
    unique_robot.registerHurtPkg(GetRobotHurtPackage());

    is_robot_inited = true;
  }
  return &unique_robot;
};

/* Private function definitions ----------------------------------------------*/
