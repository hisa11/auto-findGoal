#pragma once
/// C610 ESC ドライバ — ROS2 / Linux (SocketCAN) 対応版
/// 元コード: マイコン (mbed) 用 c610.hpp / c610.cpp を Linux 移植

#include <array>
#include <cstdint>

#include "can_motor_driver/usb_can.hpp"

// ============================================================
// 1 モーターあたりのフィードバックデータ
// ============================================================
struct C610Feedback {
  uint16_t angle{0};  // エンコーダー角度 [0–8191]
  int16_t rpm{0};     // 回転速度 [RPM]
  int16_t ampere{0};  // 実電流 [10mA単位]
  uint8_t temp{0};    // コントローラー温度 [℃]
};

// ============================================================
// C610 ドライバクラス
//   CAN ID 0x201 ~ 0x208 がフィードバック
//   0x200 (motor1-4) / 0x1FF (motor5-8) で指令送信
// ============================================================
class C610 {
 public:
  static constexpr int NUM_MOTORS = 8;
  static constexpr int16_t MAX_POWER = 10000;

  explicit C610(UsbCan* can_bus);

  /// 電流指令を送信 (0x200 と 0x1FF の 2 フレーム)
  /// 戻り値: 両フレームの送信成功なら true
  bool send_message();

  /// モーター電流を設定 (id: 1–8)
  void set_power(int id, int16_t power);

  /// 全モーター停止 (電流 0)
  void stop();

  /// 非ブロッキングで受信キューをすべて処理
  void param_update();

  // ---- ゲッター (id: 1–8) ----
  int16_t get_rpm(int id) const { return param_[id - 1].rpm; }
  uint16_t get_angle(int id) const { return param_[id - 1].angle; }
  int16_t get_ampere(int id) const { return param_[id - 1].ampere; }
  uint8_t get_temp(int id) const { return param_[id - 1].temp; }
  int16_t get_send_power(int id) const { return send_power_[id - 1]; }

  /// 直近の send_message() での各フレーム送信結果
  bool get_last_ok_200() const { return last_ok_200_; }
  bool get_last_ok_1ff() const { return last_ok_1ff_; }

 private:
  UsbCan* can_bus_;
  int16_t send_power_[NUM_MOTORS]{};
  C610Feedback param_[NUM_MOTORS]{};
  bool last_ok_200_{false};
  bool last_ok_1ff_{false};
};
