#include "ins_pwr_limiter.hpp"
#include "power_limiter.hpp"

static const hw_pwr_limiter::PowerLimiterStaticParams kStaticParams = {
    .wheel_motor_params =
        {
            .k1 = 0.23f,                        ///< Iω 项系数
            .k2 = 0.11f,                        ///< I^2 项系数
            .k3 = 0.15f,                        ///< fabsf(ω) 项系数
            .kp = 1800.0f * (20.0f / 16384.0f), ///< 速度环比例系数
            .out_limit = 20.0f,                 ///< 输出限幅，单位：A
            .motor_cnt = 4,                     ///< 电机数量
        },
    .steering_motor_params =
        {
            .k1 = 0.65f,       ///< Iω 项系数
            .k2 = 5.0f,        ///< I^2 项系数
            .k3 = 0.15f,       ///< fabsf(ω) 项系数
            .out_limit = 3.0f, ///< 输出限幅，单位：A
            .motor_cnt = 4,    ///< 电机数量
        },
    .p_bias = 5.7647f, ///< 底盘静息功率，单位：W
    .p_steering_ratio =
        0.5f, ///< 分配给舵电机的功率比例，非舵轮底盘为 0，范围：[0, 1]
};

hw_pwr_limiter::PowerLimiter unique_pwr_limiter =
    hw_pwr_limiter::PowerLimiter(kStaticParams);

hw_pwr_limiter::PowerLimiter *GetPwrLimiter() { return &unique_pwr_limiter; }