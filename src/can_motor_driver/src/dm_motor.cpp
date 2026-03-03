#include "can_motor_driver/dm_motor.hpp"

#include <cstring>

DMMotor::DMMotor(UsbCan* can_bus, uint32_t can_id, Mode mode)
    : can_bus_(can_bus),
      can_id_(can_id),
      mode_(mode),
      target_val1_(0.0f),
      target_val2_(0.0f) {}

// モータ有効化: ID=can_id_, data={0xFF*7, 0xFC}
bool DMMotor::enable() {
  if (!can_bus_) return false;
  struct can_frame frame;
  frame.can_id = can_id_;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFC;
  return can_bus_->send(frame);
}

// モータ無効化: ID=can_id_, data={0xFF*7, 0xFD}
bool DMMotor::disable() {
  if (!can_bus_) return false;
  struct can_frame frame;
  frame.can_id = can_id_;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFD;
  return can_bus_->send(frame);
}

void DMMotor::set_target_velocity(float vel) { target_val1_ = vel; }
void DMMotor::set_target_position(float pos, float max_vel) {
  target_val1_ = pos;
  target_val2_ = max_vel;
}

// STMの writeParameter(reg_addr, uint32_t) と同じフォーマット
// CAN ID=0x7FF, data[0-1]=motor_id, data[2]=0x55, data[3]=reg_addr,
// data[4-7]=value(LE)
bool DMMotor::write_parameter(uint8_t reg_addr, uint32_t value) {
  if (!can_bus_) return false;
  struct can_frame frame;
  frame.can_id = 0x7FF;
  frame.can_dlc = 8;
  frame.data[0] = can_id_ & 0xFF;
  frame.data[1] = (can_id_ >> 8) & 0xFF;
  frame.data[2] = 0x55;
  frame.data[3] = reg_addr;
  frame.data[4] = value & 0xFF;
  frame.data[5] = (value >> 8) & 0xFF;
  frame.data[6] = (value >> 16) & 0xFF;
  frame.data[7] = (value >> 24) & 0xFF;
  return can_bus_->send(frame);
}

bool DMMotor::set_control_mode(uint8_t mode) {
  return write_parameter(DM_RID_CTRL_MODE, static_cast<uint32_t>(mode));
}

bool DMMotor::send() {
  if (!can_bus_) return false;
  struct can_frame frame;

  if (mode_ == Mode::VELOCITY) {
    // 速度モード: CAN ID = 0x200 + モータID
    frame.can_id = 0x200 + can_id_;
    frame.can_dlc = 4;
    std::memcpy(frame.data, &target_val1_, sizeof(float));
  } else if (mode_ == Mode::POSITION) {
    // 位置モード: CAN ID = 0x100 + モータID
    frame.can_id = 0x100 + can_id_;
    frame.can_dlc = 8;
    std::memcpy(&frame.data[0], &target_val1_, sizeof(float));
    std::memcpy(&frame.data[4], &target_val2_, sizeof(float));
  } else {
    return false;
  }
  return can_bus_->send(frame);
}
