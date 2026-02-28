#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>  // エラー番号を扱うために追加
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

class MotorNode : public rclcpp::Node {
 public:
  MotorNode() : Node("motor_node"), can_socket_(-1) {
    // --- SocketCANの初期化 ---
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "ソケット作成失敗");
      return;
    }

    std::strcpy(ifr_.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "インターフェース取得失敗");
      return;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "バインド失敗。can0の状態を確認してください");
      return;
    }

    // 送信データの初期化（すべて0）
    std::memset(pwm_outputs_, 0, sizeof(pwm_outputs_));

    // --- ROS2の設定 ---
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&MotorNode::joy_callback, this, std::placeholders::_1));

    // 30msごとに送信
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(30),
                                std::bind(&MotorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "CAN送信ノード（30ms周期）が起動しました");
  }

  ~MotorNode() {
    if (can_socket_ >= 0) close(can_socket_);
  }

 private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    float forward = msg->axes[1];
    float turn = msg->axes[0];

    // 足回りのPWM計算
    pwm_outputs_[0] = (int16_t)((forward + turn) * 4000);  // 左前
    pwm_outputs_[1] = (int16_t)((forward - turn) * 4000);  // 右前
    pwm_outputs_[2] = (int16_t)((forward + turn) * 4000);  // 左後
    pwm_outputs_[3] = (int16_t)((forward - turn) * 4000);  // 右後
  }

  void timer_callback() {
    struct can_frame frame;  // <-- ここを frame と定義
    frame.can_id = 0x01;
    frame.can_dlc = 8;

    std::memcpy(frame.data, pwm_outputs_, sizeof(pwm_outputs_));

    // 送信処理（&frame を使用するように修正）
    if (write(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
      // エラー理由を具体的に表示するように変更
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "CAN送信失敗: %s", std::strerror(errno));
    }
  }

  int can_socket_;
  struct sockaddr_can addr_;
  struct ifreq ifr_;
  int16_t pwm_outputs_[4];
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
