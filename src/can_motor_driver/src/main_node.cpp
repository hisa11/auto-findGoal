#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// 作成した自作ライブラリを読み込む
#include "can_motor_driver/dm_motor.hpp"
#include "can_motor_driver/usb_can.hpp"

// ============================================================
// ★ チューニング用パラメータ
// ============================================================
// [pitch] motor1 の固定角度
static constexpr float PITCH_TARGET_DEG = 45.0f;  // 上下角 [度]
static constexpr float PITCH_MAX_VEL = 2.0f;      // pitch 移動速度 [rad/s]
static constexpr float GEAR_RATIO = 15.56f;       // ギア比

// [yaw] motor2 の追従制御
static constexpr float IMAGE_WIDTH = 640.0f;  // カメラ画像幅 [px]
static constexpr float IMAGE_CENTER_X = IMAGE_WIDTH / 2.0f;  // 320px
static constexpr float YAW_KP = 0.02f;      // 比例ゲイン [rad/s per px]
static constexpr float YAW_MAX_VEL = 3.0f;  // yaw 最大速度 [rad/s]
static constexpr float TOLERANCE_RATIO =
    0.10f;  // デッドバンド = bbox幅 × この値
            // (0.10 → ±10% = ゴール幅の20%以内なら停止)
static constexpr double GOAL_TIMEOUT_SEC =
    0.5;  // ゴール消失後に停止するまでの時間 [s]
// ============================================================

static float deg2rad(float deg) { return deg * (3.14159265f / 180.0f); }

class MainControllerNode : public rclcpp::Node {
 public:
  MainControllerNode()
      : Node("main_controller_node"),
        startup_count_(0),
        goal_cx_(IMAGE_CENTER_X),
        goal_width_(0.0f),
        goal_detected_(false) {
    can_bus_ = std::make_shared<UsbCan>("can0");
    if (!can_bus_->is_open()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "CAN(can0)を開けません。インターフェースを確認してください。");
      return;
    }

    // motor1: 上下角 (位置制御)
    // motor2: 左右   (速度制御)
    motor1_ = std::make_shared<DMMotor>(can_bus_.get(), 0x01,
                                        DMMotor::Mode::POSITION);
    motor2_ = std::make_shared<DMMotor>(can_bus_.get(), 0x02,
                                        DMMotor::Mode::VELOCITY);

    // pitch の目標角を設定（ギア比込み）
    motor1_->set_target_position(deg2rad(PITCH_TARGET_DEG * GEAR_RATIO),
                                 PITCH_MAX_VEL);
    motor2_->set_target_velocity(0.0f);

    // YOLO ゴール情報のサブスクライバー
    // x=ゴール中心x[px], y=ゴール中心y[px], z=ゴール幅[px]
    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/yolo/goal_info", 10,
        std::bind(&MainControllerNode::goal_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&MainControllerNode::timer_callback, this));

    last_goal_time_ = this->now();

    // CAN0 ステータスパブリッシャー
    can0_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/can_status/can0", rclcpp::QoS(1));

    // 500ms ごとに CAN0 状態をパブリッシュ
    can_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MainControllerNode::publish_can0_status, this));

    RCLCPP_INFO(this->get_logger(),
                "ゴール追従モード起動。pitch=%.1f度 (固定), yaw=VELOCITY追従",
                PITCH_TARGET_DEG);
  }

 private:
  void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    goal_cx_ = static_cast<float>(msg->x);
    goal_width_ = static_cast<float>(msg->z);
    goal_detected_ = true;
    last_goal_time_ = this->now();

    // ---- yaw 比例制御 ----
    float error_x = goal_cx_ - IMAGE_CENTER_X;
    float dead_band = goal_width_ * TOLERANCE_RATIO;

    float yaw_vel = 0.0f;
    if (std::abs(error_x) > dead_band) {
      // 正値→左回転なので、ゴールが右(error>0)なら負速度(右回転)
      yaw_vel = -YAW_KP * error_x;
      yaw_vel = std::clamp(yaw_vel, -YAW_MAX_VEL, YAW_MAX_VEL);
    }
    motor2_->set_target_velocity(yaw_vel);
  }

  void timer_callback() {
    if (!can_bus_->is_open()) return;

    // 起動シーケンス (30ms × カウント)
    // count=2 : motor1 に位置制御モードを書き込む
    // count=4 : motor1 enable
    // count=6 : motor2 に速度制御モードを書き込む (ファームウェア設定の上書き)
    // count=8 : motor2 enable
    // count<40: 有効化待機
    if (startup_count_ < 40) {
      if (startup_count_ == 2) {
        motor1_->set_control_mode(DM_POSITION_MODE);
        RCLCPP_INFO(this->get_logger(), "motor1 POSITION_MODE 設定 (ID=0x01)");
      } else if (startup_count_ == 4) {
        motor1_->enable();
        RCLCPP_INFO(this->get_logger(), "motor1 enable 送信 (ID=0x01)");
      } else if (startup_count_ == 6) {
        motor2_->set_control_mode(DM_VELOCITY_MODE);
        RCLCPP_INFO(this->get_logger(), "motor2 VELOCITY_MODE 設定 (ID=0x02)");
      } else if (startup_count_ == 8) {
        motor2_->enable();
        RCLCPP_INFO(this->get_logger(), "motor2 enable 送信 (ID=0x02)");
      }
      can_bus_->drain();
      startup_count_++;
      return;
    }

    if (startup_count_ == 40) {
      RCLCPP_INFO(this->get_logger(), "制御開始！YOLOゴールを追従します。");
    }

    // ゴールが一定時間見えなくなったら yaw 停止
    double elapsed = (this->now() - last_goal_time_).seconds();
    if (elapsed > GOAL_TIMEOUT_SEC && goal_detected_) {
      motor2_->set_target_velocity(0.0f);
      goal_detected_ = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "ゴール未検出 → yaw 停止");
    }

    can_bus_->drain();
    motor1_->send();
    motor2_->send();

    startup_count_++;
  }

  void publish_can0_status() {
    std_msgs::msg::Bool msg;
    msg.data = can_bus_ && can_bus_->is_open();
    can0_status_pub_->publish(msg);
  }

  std::shared_ptr<UsbCan> can_bus_;
  std::shared_ptr<DMMotor> motor1_;
  std::shared_ptr<DMMotor> motor2_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr can_status_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can0_status_pub_;

  int startup_count_;
  float goal_cx_;
  float goal_width_;
  bool goal_detected_;
  rclcpp::Time last_goal_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainControllerNode>());
  rclcpp::shutdown();
  return 0;
}
