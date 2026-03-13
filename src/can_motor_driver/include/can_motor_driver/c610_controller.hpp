#pragma once
/// c610_controller.hpp
/// C610 ESC 4軸 PID コントローラー (mbed 実績 PID ライブラリ使用)
/// CAN 送受信は行わない。
///   - main_node が feed_rpm() で受信 RPM を渡す
///   - compute() で PID 計算
///   - get_power() で電流指令値を読み取る

#include <array>
#include <atomic>
#include <cstdint>

#include "can_motor_driver/pid.hpp"

// ============================================================
namespace c610_param {
static constexpr int NUM_MOTORS = 7;          // C610 モーター数 (1−7)
static constexpr int16_t MAX_POWER = 10000;
}  // namespace c610_param

// ============================================================
/// モーター 1 軸分の PID パラメータ宣言
/// main_node のコンストラクタで軸ごとに指定する
// ============================================================
struct C610MotorParam {
  double kp{0.6};
  double ki{0.0};
  double kd{0.0};
  double output_min{-static_cast<double>(c610_param::MAX_POWER)};
  double output_max{static_cast<double>(c610_param::MAX_POWER)};
  PID::Mode mode{
      PID::Mode::VELOCITY};  // VELOCITY = 速度型(増分型) — RM モータ推奨
};

// ============================================================
class C610Controller {
 public:
  explicit C610Controller(
      const std::array<C610MotorParam, c610_param::NUM_MOTORS>& params);

  /// モーターごとの目標 RPM セット (motor_idx: 0-origin)
  void set_target_rpm(int motor_idx, int rpm);

  /// CAN フィードバックから RPM を渡す (motor_idx: 0-origin)
  void feed_rpm(int motor_idx, int16_t actual_rpm);

  /// PID 演算。制御ループ毎に呼ぶ (dt 単位: 秒)
  void compute(double dt);

  /// 各モーターの電流指令値 (motor_idx: 0-origin)
  int16_t get_power(int motor_idx) const { return power_[motor_idx]; }

  /// 各モーターの現在の目標 RPM (motor_idx: 0-origin)
  int get_target_rpm(int motor_idx) const {
    if (motor_idx < 0 || motor_idx >= c610_param::NUM_MOTORS) return 0;
    return target_rpm_[motor_idx].load();
  }

  /// 全モーター目標が全て 0 の時 true
  bool is_stopped() const {
    for (int i = 0; i < c610_param::NUM_MOTORS; ++i)
      if (target_rpm_[i].load() != 0) return false;
    return true;
  }

 private:
  std::atomic<int> target_rpm_[c610_param::NUM_MOTORS]{};
  std::atomic<int16_t> actual_rpm_[c610_param::NUM_MOTORS]{};
  std::array<PID, c610_param::NUM_MOTORS> pids_;
  int16_t power_[c610_param::NUM_MOTORS]{};
};
