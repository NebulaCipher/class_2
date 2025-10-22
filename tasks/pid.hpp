#pragma once
#include <algorithm>

struct PidParams {
    float kp, ki, kd;
    float maxOut, minOut;
};

class Pid {
public:
    Pid(PidParams params);
    ~Pid() = default;
    float pidCalc(float ref, float fdb, int flag);

private:
    PidParams params_;
    float integral_;
    float last_error_;
};
