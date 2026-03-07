/// C610 ESC ドライバ — ROS2 / Linux (SocketCAN) 対応版
/// 元コード: マイコン (mbed) 用 c610.cpp を Linux 移植

#include "can_motor_driver/c610.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cstring>

C610::C610(UsbCan* can_bus) : can_bus_(can_bus) {
  std::fill(std::begin(send_power_), std::end(send_power_), int16_t{0});
}

// ============================================================
// 電流指令送信
//   0x200 : motor 1-4 (buf[0..7])
//   0x1FF : motor 5-8 (buf[8..15])
// ============================================================
bool C610::send_message() {
  uint8_t buf[16];
  for (int i = 0; i < NUM_MOTORS; ++i) {
    buf[i * 2] = static_cast<uint8_t>((send_power_[i] >> 8) & 0xFF);
    buf[i * 2 + 1] = static_cast<uint8_t>(send_power_[i] & 0xFF);
  }

  bool success = true;

  // motor 1-4 → ID 0x200
  {
    struct can_frame frame{};
    frame.can_id = 0x200;
    frame.can_dlc = 8;
    std::memcpy(frame.data, buf, 8);
    if (!can_bus_->send(frame)) {
      success = false;
    }
  }

  // motor 5-8 → ID 0x1FF
  {
    struct can_frame frame{};
    frame.can_id = 0x1FF;
    frame.can_dlc = 8;
    std::memcpy(frame.data, buf + 8, 8);
    if (!can_bus_->send(frame)) {
      success = false;
    }
  }

  return success;
}

// ============================================================
// 電流設定 (id: 1-8)
// ============================================================
void C610::set_power(int id, int16_t power) {
  if (id < 1 || id > NUM_MOTORS) return;
  if (power > MAX_POWER)
    send_power_[id - 1] = MAX_POWER;
  else if (power < -MAX_POWER)
    send_power_[id - 1] = -MAX_POWER;
  else
    send_power_[id - 1] = power;
}

// ============================================================
// 全停止
// ============================================================
void C610::stop() {
  std::fill(std::begin(send_power_), std::end(send_power_), int16_t{0});
}

// ============================================================
// 受信フィードバックを非ブロッキングで全処理
//   C610 フィードバック CAN ID: 0x201(motor1) ~ 0x208(motor8)
//   データ: [angle_H, angle_L, rpm_H, rpm_L, amp_H, amp_L, temp, 0x00]
// ============================================================
void C610::param_update() {
  struct can_frame msg;
  while (can_bus_->recv(msg)) {
    // 標準フレーム & 8 バイト & ID 0x201-0x208 のみ処理
    if ((msg.can_id & CAN_EFF_FLAG) == 0 &&  // 標準 ID
        msg.can_dlc == 8 && msg.can_id >= 0x201 && msg.can_id <= 0x208) {
      int idx = static_cast<int>(msg.can_id) - 0x201;
      param_[idx].angle = static_cast<uint16_t>(msg.data[0] << 8 | msg.data[1]);
      param_[idx].rpm = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
      param_[idx].ampere = static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
      param_[idx].temp = msg.data[6];
    }
  }
}
