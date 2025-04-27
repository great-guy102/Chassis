#ifndef CHASSIS_INSTANCE_INS_PWR_LIMITER_HPP_
#define CHASSIS_INSTANCE_INS_PWR_LIMITER_HPP_
#include "power_limiter.hpp"

namespace hw_pwr_limiter = hello_world::power_limiter;

hw_pwr_limiter::PowerLimiter *GetPwrLimiter();
#endif /* CHASSIS_INSTANCE_INS_PWR_LIMITER_HPP_ */