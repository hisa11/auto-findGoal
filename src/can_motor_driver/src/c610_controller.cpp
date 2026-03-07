#include "can_motor_driver/c610_controller.hpp"

#include <cstdint>

using namespace c610_param;

// ============================================================
// コンストラクタ: 各軸の PID を params で初期化
// ============================================================
C610Controller::C610Controller(
    const std::array<C610MotorParam, NUM_MOTORS>& params)
    : pids_{PID(params[0].kp, params[0].ki, params[0].kd, params[0].mode),
            PID(params[1].kp, params[1].ki, params[1].kd, params[1].mode),
            PID(params[2].kp, params[2].ki, params[2].kd, params[2].mode),
            PID(params[3].kp, params[3].ki, params[3].kd, params[3].mode)} {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pids_[i].set_output_limits(params[i].output_min, params[i].output_max);
  }
  for (auto& r : actual_rpm_) r.store(0);
  for (auto& p : power_) p = 0;
}

void C610Controller::set_target_rpm(int rpm) {
  target_rpm_.store(rpm);
  // rpm==0 でも PID をリセットしない — 速度型 PID が能動的にブレーキをかける
}

void C610Controller::feed_rpm(int motor_idx, int16_t actual_rpm) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS) return;
  actual_rpm_[motor_idx].store(actual_rpm);
}

void C610Controller::compute(double dt) {
  // target=0 を含む全ての目標値に対して PID を実行し、能動的に追従する
  const double target = static_cast<double>(target_rpm_.load());
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pids_[i].set_dt(dt);
    pids_[i].set_goal(target);
    double actual = static_cast<double>(actual_rpm_[i].load());
    double out = pids_[i].do_pid(actual);
    power_[i] = static_cast<int16_t>(out);
  }
}
