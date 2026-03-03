#pragma once
#include "can_motor_driver/usb_can.hpp"
#include <cstdint>

class BaseMotor {
public:
    BaseMotor(UsbCan* can_bus, uint32_t can_id);
    void set_pwm(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
    bool send();
private:
    UsbCan* can_bus_;
    uint32_t can_id_;
    int16_t pwm_[4];
};
