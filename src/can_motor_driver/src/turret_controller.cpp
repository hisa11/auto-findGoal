#include "can_motor_driver/turret_controller.hpp"

using namespace turret_param;

static inline float deg2rad(float d) { return d * (3.14159265f / 180.0f); }

TurretController::TurretController() = default;

// ============================================================
// 外部入力
// ============================================================
void TurretController::update_goal(float cx, float bbox_w, int64_t stamp_ns) {
  float err = cx - IMAGE_CENTER_X;
  float dead_band = bbox_w * TOLERANCE_RATIO;
  float vel = 0.0f;
  if (std::abs(err) > dead_band)
    vel = std::clamp(-YAW_KP * err, -YAW_MAX_VEL, YAW_MAX_VEL);

  std::lock_guard<std::mutex> lk(mtx_);
  yaw_vel_ = vel;
  goal_detected_ = true;
  last_goal_ns_ = stamp_ns;
}

// ============================================================
// 起動シーケンス 1 ステップ
// ============================================================
StartupCmd TurretController::tick_startup() {
  StartupCmd cmd = StartupCmd::NONE;
  switch (startup_count_) {
    case CNT_PITCH_MODE:
      cmd = StartupCmd::PITCH_SET_MODE;
      break;
    case CNT_PITCH_ENABLE:
      cmd = StartupCmd::PITCH_ENABLE;
      break;
    case CNT_YAW_MODE:
      cmd = StartupCmd::YAW_SET_MODE;
      break;
    case CNT_YAW_ENABLE:
      cmd = StartupCmd::YAW_ENABLE;
      break;
    default:
      cmd = StartupCmd::NONE;
      break;
  }
  ++startup_count_;
  return cmd;
}

// ============================================================
// 通常制御ループ
// ============================================================
void TurretController::step(int64_t now_ns) {
  std::lock_guard<std::mutex> lk(mtx_);

  // --- yaw 角度の積分 ---
  if (last_step_ns_ != 0) {
    float dt = static_cast<float>(now_ns - last_step_ns_) * 1e-9f;
    yaw_angle_deg_ += yaw_vel_ * dt * (180.0f / 3.14159265f);
    // ±90° でクランプし、限界方向への速度を止める
    if (yaw_angle_deg_ >= YAW_MAX_DEG) {
      yaw_angle_deg_ = YAW_MAX_DEG;
      if (yaw_vel_ > 0.0f) yaw_vel_ = 0.0f;
    } else if (yaw_angle_deg_ <= YAW_MIN_DEG) {
      yaw_angle_deg_ = YAW_MIN_DEG;
      if (yaw_vel_ < 0.0f) yaw_vel_ = 0.0f;
    }
  }
  last_step_ns_ = now_ns;

  // --- ゴールタイムアウト ---
  if (!goal_detected_) return;
  double elapsed = static_cast<double>(now_ns - last_goal_ns_) * 1e-9;
  if (elapsed > GOAL_TIMEOUT_S) {
    yaw_vel_ = 0.0f;
    goal_detected_ = false;
  }
}

// ============================================================
// ゲッター
// ============================================================
float TurretController::get_yaw_velocity() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return yaw_vel_;
}

float TurretController::get_yaw_angle_deg() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return yaw_angle_deg_;
}

float TurretController::get_pitch_rad() const {
  return deg2rad(PITCH_TARGET_DEG * GEAR_RATIO);
}
