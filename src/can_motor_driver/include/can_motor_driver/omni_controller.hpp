#pragma once
/// omni_controller.hpp
/// 4輪オムニ逆運動学 + cmd_vel ウォッチドッグ
/// CAN 送受信は行わない。main_node が ERPM 値を読み取り CAN 送信する。

#include <atomic>
#include <cmath>
#include <cstdint>

// ============================================================
// チューニング定数
// ============================================================
namespace omni_param {
static constexpr uint8_t ID_FL = 12;
static constexpr uint8_t ID_FR = 13;
static constexpr uint8_t ID_RL = 10;
static constexpr uint8_t ID_RR = 11;
static constexpr double ROBOT_RADIUS = 0.15;  // 旋回半径 [m]
static constexpr double MAX_SPEED_MPS = 1.5;  // 最大直動速度 [m/s]
static constexpr int32_t MAX_ERPM = 30000;    // 最大 ERPM
static constexpr uint8_t COMM_SET_RPM = 3;    // VESC CAN コマンド
static constexpr double CMD_TIMEOUT_S = 0.2;  // cmd_vel タイムアウト [s]
}  // namespace omni_param

// ============================================================
class OmniController {
 public:
  OmniController();

  /// /cmd_vel 受信時に呼ぶ。vx[m/s], vy[m/s], wz[rad/s]
  void update_cmd(double vx, double vy, double wz, int64_t stamp_ns);

  /// 制御ループ毎に呼ぶ。タイムアウト時は ERPM を 0 にリセットする
  void check_watchdog(int64_t now_ns);

  /// 各ホイールの目標 ERPM (0=FL 1=FR 2=RL 3=RR)
  int32_t get_erpm(int wheel) const { return erpm_[wheel].load(); }

 private:
  static int32_t mps_to_erpm(double v);

  std::atomic<int32_t> erpm_[4];
  std::atomic<int64_t> last_cmd_ns_{0};
};
