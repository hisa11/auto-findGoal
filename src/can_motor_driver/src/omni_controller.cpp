#include "can_motor_driver/omni_controller.hpp"

#include <algorithm>

OmniController::OmniController() {
  for (auto& e : erpm_) e.store(0);
}

int32_t OmniController::mps_to_erpm(double v) {
  using namespace omni_param;
  return static_cast<int32_t>(std::clamp(v / MAX_SPEED_MPS * MAX_ERPM,
                                         -static_cast<double>(MAX_ERPM),
                                         static_cast<double>(MAX_ERPM)));
}

void OmniController::update_cmd(double vx, double vy, double wz,
                                int64_t stamp_ns) {
  using namespace omni_param;
  erpm_[0].store(mps_to_erpm(+vx - vy - wz * ROBOT_RADIUS));  // FL
  erpm_[1].store(mps_to_erpm(-vx - vy - wz * ROBOT_RADIUS));  // FR
  erpm_[2].store(mps_to_erpm(+vx + vy - wz * ROBOT_RADIUS));  // RL
  erpm_[3].store(mps_to_erpm(-vx + vy - wz * ROBOT_RADIUS));  // RR
  last_cmd_ns_.store(stamp_ns);
}

void OmniController::check_watchdog(int64_t now_ns) {
  int64_t last = last_cmd_ns_.load();
  if (last == 0) return;
  double elapsed = static_cast<double>(now_ns - last) * 1e-9;
  if (elapsed > omni_param::CMD_TIMEOUT_S) {
    for (auto& e : erpm_) e.store(0);
  }
}
