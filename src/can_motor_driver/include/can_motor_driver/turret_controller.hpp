#pragma once
/// turret_controller.hpp
/// 砲塔 (DM モータ) 状態管理 + YOLO ゴール追従
/// CAN 送受信は行わない。
///   - main_node が update_goal() で YOLO 座標を渡す
///   - step() でタイムアウト判定・yaw 速度計算
///   - 起動シーケンス判定を main_node に問い合わせる

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

// ============================================================
// チューニング定数
// ============================================================
namespace turret_param {
static constexpr float PITCH_TARGET_DEG = 45.0f;  // 上下固定角 [deg]
static constexpr float PITCH_MAX_VEL = 2.0f;      // pitch 最大速 [rad/s]
static constexpr float GEAR_RATIO = 15.56f;       // ギア比
static constexpr float IMAGE_WIDTH = 640.0f;
static constexpr float IMAGE_CENTER_X = IMAGE_WIDTH / 2.0f;
static constexpr float YAW_KP = 0.02f;           // yaw 比例ゲイン
static constexpr float YAW_MAX_VEL = 3.0f;       // [rad/s]
static constexpr float TOLERANCE_RATIO = 0.10f;  // デッドバンド比
static constexpr double GOAL_TIMEOUT_S = 0.5;    // ゴール消失タイムアウト [s]
// 起動シーケンス: 各カウントで何を行うか (main_node 側で参照)
static constexpr int CNT_PITCH_MODE = 2;
static constexpr int CNT_PITCH_ENABLE = 4;
static constexpr int CNT_YAW_MODE = 6;
static constexpr int CNT_YAW_ENABLE = 8;
static constexpr int CNT_STARTUP_DONE = 40;
}  // namespace turret_param

// ============================================================
/// 起動シーケンスの 1 ステップで main_node が行うべき操作
enum class StartupCmd {
  NONE,
  PITCH_SET_MODE,
  PITCH_ENABLE,
  YAW_SET_MODE,
  YAW_ENABLE,
};

// ============================================================
class TurretController {
 public:
  TurretController();

  // ---- 外部からのデータ入力 ----

  /// YOLO ゴール情報 (cx[px], bbox_w[px], stamp_ns)
  void update_goal(float cx, float bbox_w, int64_t stamp_ns);

  // ---- 制御ループから呼ぶ ----

  /// 起動カウントをインクリメントして起動コマンドを返す
  /// startup_done() が false の間だけ呼ぶ
  StartupCmd tick_startup();

  /// ゴールタイムアウト判定 + yaw 速度更新。startup_done() 後に呼ぶ
  void step(int64_t now_ns);

  // ---- ゲッター (main_node が読み取って DMMotor/CAN 操作に使う) ----

  bool startup_done() const {
    return startup_count_ >= turret_param::CNT_STARTUP_DONE;
  }
  int startup_count() const { return startup_count_; }
  /// DM モータ不使用時に起動シーケンスを即完了させる
  void skip_startup() { startup_count_ = turret_param::CNT_STARTUP_DONE; }
  float get_yaw_velocity() const;
  float get_pitch_rad() const;  // ギア比込み [rad]
  float get_pitch_max_vel() const { return turret_param::PITCH_MAX_VEL; }

 private:
  int startup_count_{0};
  mutable std::mutex mtx_;
  float yaw_vel_{0.0f};
  bool goal_detected_{false};
  int64_t last_goal_ns_{0};
};
