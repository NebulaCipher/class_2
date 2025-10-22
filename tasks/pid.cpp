#include "pid.hpp"
#include <cmath>
Pid::Pid(PidParams params)
    : params_(params), integral_(0.0f), last_error_(0.0f)
{
}

float Pid::pidCalc(float ref, float fdb, int flag)
{
    float error = ref - fdb;
if(flag == 1){  // 位置环角度误差处理, 速度环忽略
    while (error > M_PI)  error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;
}
    integral_ += error;

    float derivative = error - last_error_;
    float output = params_.kp * error + params_.ki * integral_ + params_.kd * derivative;

    if (output > params_.maxOut) output = params_.maxOut;
    if (output < params_.minOut) output = params_.minOut;

    last_error_ = error;
    return output;
}
