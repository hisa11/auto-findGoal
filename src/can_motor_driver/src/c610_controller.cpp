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
            PID(params[3].kp, params[3].ki, params[3].kd, params[3].mode),
            PID(params[4].kp, params[4].ki, params[4].kd, params[4].mode),
            PID(params[5].kp, params[5].ki, params[5].kd, params[5].mode),
            PID(params[6].kp, params[6].ki, params[6].kd, params[6].mode)} {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pids_[i].set_output_limits(params[i].output_min, params[i].output_max);
    target_rpm_[i].store(0);
  }
  for (auto& r : actual_rpm_) r.store(0);
  for (auto& p : power_) p = 0;
}

void C610Controller::set_target_rpm(int motor_idx, int rpm) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS) return;
  target_rpm_[motor_idx].store(rpm);
}

void C610Controller::feed_rpm(int motor_idx, int16_t actual_rpm) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS) return;
  actual_rpm_[motor_idx].store(actual_rpm);
}

void C610Controller::compute(double dt) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pids_[i].set_dt(dt);
    double goal   = static_cast<double>(target_rpm_[i].load());
    double actual = static_cast<double>(actual_rpm_[i].load());
    pids_[i].set_goal(goal);
    double out = pids_[i].do_pid(actual);
    power_[i] = static_cast<int16_t>(out);
  }
}
