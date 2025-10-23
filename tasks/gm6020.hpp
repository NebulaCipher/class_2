#ifndef _GM6020_H_
#define _GM6020_H_
#pragma once
#include "main.h"

class GM6020 {
public:
    GM6020(uint32_t id);
    ~GM6020() = default;

    bool decode(uint8_t* data);
    bool encode(uint8_t* data);

    void setInput(float input) { input_ = input; }
    float vel() const { return vel_; }
    float angle() const { return angle_; }
    uint32_t rxId() const { return 0x207; } 

private:
    uint32_t id_;
    float input_;
    float angle_;
    float vel_;
    float current_;
    float temp_;
};
#endif // _GM6020_H_