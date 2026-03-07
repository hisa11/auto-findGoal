/// c610_node.cpp
/// ロボットマスターモータ (C610 ESC) 制御ノード
///
/// トピック:
///   sub  /c610/target_rpm  std_msgs/msg/Int32   全モーター目標 RPM
///   pub  /can_status/can0  std_msgs/msg/Bool    CAN 送信成否
///
/// PS4 コントローラー:
///   △ (triangle) → 8000 RPM で全モーター回転
///   × (cross)    → 0 RPM (停止)
/// を gamepad.js 側が /c610/target_rpm に送信し、このノードが受け取る。

#include <algorithm>
#include <array>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "can_motor_driver/c610.hpp"
#include "can_motor_driver/usb_can.hpp"

// ============================================================
// チューニング用定数
// ============================================================
static constexpr int NUM_MOTORS = 4;   // 接続モーター数
static constexpr int CONTROL_MS = 30;  // 制御ループ周期 [ms]
static constexpr double CONTROL_HZ = 1000.0 / CONTROL_MS;  // ≈ 33.3 Hz
static constexpr int TARGET_RPM_DEF = 0;                   // 起動時目標 RPM
static constexpr double PID_KP = 1.0;                      // 比例ゲイン
static constexpr double PID_KI = 0.05;                     // 積分ゲイン
static constexpr double PID_KD = 0.2;                      // 微分ゲイン
static constexpr double INTEGRAL_LIMIT = 5000.0;  // 積分ワインドアップ制限
static constexpr int16_t MAX_POWER = C610::MAX_POWER;

// ============================================================
// 単軸 PID 構造体
// ============================================================
struct Pid {
  double kp, ki, kd;
  double integral{0.0};
  double prev_error{0.0};

  int16_t compute(double target, double actual, double dt) {
    double error = target - actual;
    integral =
        std::clamp(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    double derivative = (dt > 1e-9) ? (error - prev_error) / dt : 0.0;
    prev_error = error;
    double out = kp * error + ki * integral + kd * derivative;
    return static_cast<int16_t>(std::clamp(out, static_cast<double>(-MAX_POWER),
                                           static_cast<double>(MAX_POWER)));
  }

  void reset() {
    integral = 0.0;
    prev_error = 0.0;
  }
};

// ============================================================
// C610 コントロールノード
// ============================================================
class C610Node : public rclcpp::Node {
 public:
  C610Node() : Node("c610_node"), target_rpm_(TARGET_RPM_DEF) {
    // CAN バス初期化
    can_bus_ = std::make_shared<UsbCan>("can0");
    if (!can_bus_->is_open()) {
      RCLCPP_ERROR(get_logger(),
                   "[C610] CAN(can0) "
                   "を開けません。インターフェースを確認してください。");
    }

    // C610 ドライバ
    c610_ = std::make_shared<C610>(can_bus_.get());

    // PID 初期化
    for (auto& pid : pids_) {
      pid.kp = PID_KP;
      pid.ki = PID_KI;
      pid.kd = PID_KD;
    }

    // ---- ROS トピック ----
    // 目標 RPM の受信
    target_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/c610/target_rpm", rclcpp::QoS(1),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          target_rpm_ = msg->data;
          // 目標変更時は積分をリセット（過渡応答改善）
          if (target_rpm_ == 0) {
            for (auto& pid : pids_) pid.reset();
            c610_->stop();
          }
          RCLCPP_INFO(get_logger(), "[C610] 目標 RPM → %d", target_rpm_);
        });

    // CAN ステータスのパブリッシャー
    can_status_pub_ = create_publisher<std_msgs::msg::Bool>("/can_status/can0",
                                                            rclcpp::QoS(1));

    // 制御ループタイマー (30 ms)
    timer_ = create_wall_timer(std::chrono::milliseconds(CONTROL_MS),
                               std::bind(&C610Node::control_loop, this));

    last_time_ = now();
    RCLCPP_INFO(get_logger(),
                "[C610] ノード起動。制御周期=%d ms, モーター数=%d", CONTROL_MS,
                NUM_MOTORS);
  }

 private:
  void control_loop() {
    if (!can_bus_->is_open()) {
      publish_can_status(false);
      return;
    }

    // dt 計算
    auto now_t = now();
    double dt = (now_t - last_time_).seconds();
    last_time_ = now_t;
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / CONTROL_HZ;

    // フィードバック受信（non-blocking で全キュー処理）
    c610_->param_update();

    // PID → 電流指令
    for (int i = 1; i <= NUM_MOTORS; ++i) {
      int16_t power =
          pids_[i - 1].compute(static_cast<double>(target_rpm_),
                               static_cast<double>(c610_->get_rpm(i)), dt);
      c610_->set_power(i, power);
    }

    // 送信 (戻り値を CAN ステータスに使う)
    bool ok = c610_->send_message();
    publish_can_status(ok);
  }

  void publish_can_status(bool ok) {
    std_msgs::msg::Bool msg;
    msg.data = ok;
    can_status_pub_->publish(msg);
  }

  // ---- メンバ変数 ----
  std::shared_ptr<UsbCan> can_bus_;
  std::shared_ptr<C610> c610_;
  std::array<Pid, NUM_MOTORS> pids_;
  int target_rpm_;
  rclcpp::Time last_time_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<C610Node>());
  rclcpp::shutdown();
  return 0;
}
