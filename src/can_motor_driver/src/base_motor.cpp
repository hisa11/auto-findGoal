#include "can_motor_driver/base_motor.hpp"
#include <cstring>

BaseMotor::BaseMotor(UsbCan* can_bus, uint32_t can_id) : can_bus_(can_bus), can_id_(can_id) {
    std::memset(pwm_, 0, sizeof(pwm_));
}
void BaseMotor::set_pwm(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
    pwm_[0] = fl; pwm_[1] = fr; pwm_[2] = rl; pwm_[3] = rr;
}
bool BaseMotor::send() {
    if (!can_bus_) return false;
    struct can_frame frame;
    frame.can_id = can_id_;
    frame.can_dlc = 8;
    std::memcpy(frame.data, pwm_, sizeof(pwm_));
    return can_bus_->send(frame);
}
