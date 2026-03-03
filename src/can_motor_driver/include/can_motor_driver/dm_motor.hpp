#pragma once
#include <cstdint>

#include "can_motor_driver/usb_can.hpp"

// 制御モード (STMコードと同じ定義)
#define DM_MIT_MODE 1
#define DM_POSITION_MODE 2
#define DM_VELOCITY_MODE 3

// レジスタアドレス
#define DM_RID_CTRL_MODE 0x0A

class DMMotor {
 public:
  enum class Mode { VELOCITY, POSITION };
  DMMotor(UsbCan* can_bus, uint32_t can_id, Mode mode);

  // モータの有効化・無効化（送信前に必ずenableを呼ぶこと）
  bool enable();
  bool disable();

  // ファームウェアの制御モードをレジスタに書き込む
  // mode: DM_VELOCITY_MODE(3) or DM_POSITION_MODE(2)
  bool write_parameter(uint8_t reg_addr, uint32_t value);
  bool set_control_mode(uint8_t mode);

  void set_target_velocity(float vel);
  void set_target_position(float pos, float max_vel);
  bool send();

 private:
  UsbCan* can_bus_;
  uint32_t can_id_;  // ベースモータID（enableに使用）
  Mode mode_;
  float target_val1_;
  float target_val2_;
};
