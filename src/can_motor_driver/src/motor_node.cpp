// ============================================================
// VESC 4輪オムニドライブ ノード
//
// /cmd_vel (geometry_msgs/Twist) を購読し、
// 4輪オムニホイール逆運動学によって各 VESC へ
// SocketCAN 経由で RPM 指令を送信します。
//
// 座標系 (右手系):
//   linear.x  : 前進方向速度 [m/s]
//   linear.y  : 左方向速度   [m/s]
//   angular.z : 反時計回り旋回速度 [rad/s]
//
// ホイール配置 (上面視):
//   FL --- FR
//   |       |
//   RL --- RR
//
// オムニホイール逆運動学:
//   v_FL = +vx - vy - wz * R
//   v_FR = +vx + vy + wz * R
//   v_RL = +vx + vy - wz * R
//   v_RR = +vx - vy + wz * R
// ============================================================

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

// ============================================================
// ★ チューニング用パラメータ
// ============================================================
// VESC の CAN ID (各ホイールに割り当て)
// VESC Tool の Motor Configuration → General → Motor ID に合わせる
static constexpr uint8_t VESC_ID_FL = 1;  // 前左
static constexpr uint8_t VESC_ID_FR = 2;  // 前右
static constexpr uint8_t VESC_ID_RL = 3;  // 後左
static constexpr uint8_t VESC_ID_RR = 4;  // 後右

// ロボット中心からホイールまでの距離 (旋回半径) [m]
static constexpr double ROBOT_RADIUS = 0.15;

// MAX_SPEED_MPS [m/s] のときに送る ERPM 値
// ERPM = 機械RPM × 極ペア数 なので VESC Tool の「Max ERPM」に合わせる
static constexpr double MAX_SPEED_MPS = 1.5;  // 実機の最大直動速度 [m/s]
static constexpr int32_t MAX_ERPM = 30000;    // 上記速度に対応する ERPM

// VESC CAN コマンド ID
static constexpr uint8_t COMM_SET_RPM = 3;
// ============================================================

/// @brief 速度 [m/s] → ERPM へ変換 (線形スケーリング)
static inline int32_t mps_to_erpm(double vel_mps) {
  double erpm = vel_mps / MAX_SPEED_MPS * static_cast<double>(MAX_ERPM);
  erpm = std::clamp(erpm, -static_cast<double>(MAX_ERPM),
                    static_cast<double>(MAX_ERPM));
  return static_cast<int32_t>(erpm);
}

class VescOmniNode : public rclcpp::Node {
 public:
  VescOmniNode() : Node("vesc_omni_node"), can_socket_(-1) {
    // --- SocketCAN 初期化 ---
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "SocketCAN: ソケット作成失敗 (%s)",
                   std::strerror(errno));
      return;
    }

    std::strncpy(ifr_.ifr_name, "can1", IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
      RCLCPP_ERROR(get_logger(), "SocketCAN: インターフェース取得失敗 (%s)",
                   std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr_),
             sizeof(addr_)) < 0) {
      RCLCPP_ERROR(get_logger(),
                   "SocketCAN: バインド失敗。can1 を確認してください (%s)",
                   std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    // --- /cmd_vel 購読 (受信時は ERPM 値を更新するだけ) ---
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&VescOmniNode::cmd_callback, this, std::placeholders::_1));

    // 30ms 周期で4輪まとめて CAN 送信
    send_timer_ =
        create_wall_timer(std::chrono::milliseconds(30),
                          std::bind(&VescOmniNode::send_timer_callback, this));

    // 安全停止ウォッチドッグ: 200ms 以上 cmd_vel が来なければ ERPM を 0 に
    watchdog_timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&VescOmniNode::watchdog_callback, this));

    // CAN1 ステータスパブリッシャー
    can1_status_pub_ = create_publisher<std_msgs::msg::Bool>("/can_status/can1",
                                                             rclcpp::QoS(1));

    // 500ms ごとに CAN1 状態をパブリッシュ
    can_status_timer_ =
        create_wall_timer(std::chrono::milliseconds(500),
                          std::bind(&VescOmniNode::publish_can1_status, this));

    RCLCPP_INFO(
        get_logger(),
        "VESC 4輪オムニノード起動 (FL=%d FR=%d RL=%d RR=%d, can1, 30ms)",
        VESC_ID_FL, VESC_ID_FR, VESC_ID_RL, VESC_ID_RR);
  }

  ~VescOmniNode() {
    if (can_socket_ >= 0) {
      // 停止指令を送信してから閉じる
      send_vesc_rpm(VESC_ID_FL, 0);
      send_vesc_rpm(VESC_ID_FR, 0);
      send_vesc_rpm(VESC_ID_RL, 0);
      send_vesc_rpm(VESC_ID_RR, 0);
      close(can_socket_);
    }
  }

 private:
  // ----------------------------------------------------------
  // /cmd_vel コールバック: ERPM 値を更新するだけ (送信はタイマー)
  // ----------------------------------------------------------
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_time_ = now();

    const double vx = msg->linear.x;   // 前進 [m/s]
    const double vy = msg->linear.y;   // 左方向 [m/s]
    const double wz = msg->angular.z;  // 反時計回り [rad/s]

    // 4輪オムニ逆運動学
    target_erpm_[0] = mps_to_erpm(+vx - vy - wz * ROBOT_RADIUS);  // FL
    target_erpm_[1] = mps_to_erpm(+vx + vy + wz * ROBOT_RADIUS);  // FR
    target_erpm_[2] = mps_to_erpm(+vx + vy - wz * ROBOT_RADIUS);  // RL
    target_erpm_[3] = mps_to_erpm(+vx - vy + wz * ROBOT_RADIUS);  // RR

    RCLCPP_DEBUG(get_logger(),
                 "vx=%.2f vy=%.2f wz=%.2f → FL:%d FR:%d RL:%d RR:%d", vx, vy,
                 wz, target_erpm_[0], target_erpm_[1], target_erpm_[2],
                 target_erpm_[3]);
  }

  // ----------------------------------------------------------
  // 30ms 周期送信タイマー: 4輪まとめて CAN 送信
  // ----------------------------------------------------------
  void send_timer_callback() {
    if (can_socket_ < 0) return;
    send_vesc_rpm(VESC_ID_FL, target_erpm_[0]);
    send_vesc_rpm(VESC_ID_FR, target_erpm_[1]);
    send_vesc_rpm(VESC_ID_RL, target_erpm_[2]);
    send_vesc_rpm(VESC_ID_RR, target_erpm_[3]);
  }

  // ----------------------------------------------------------
  // 安全停止ウォッチドッグ
  // ----------------------------------------------------------
  void watchdog_callback() {
    if (can_socket_ < 0) return;
    auto elapsed = (now() - last_cmd_time_).seconds();
    if (elapsed > 0.2) {
      target_erpm_[0] = target_erpm_[1] = target_erpm_[2] = target_erpm_[3] = 0;
    }
  }

  // ----------------------------------------------------------
  // VESC CAN RPM 指令フレーム送信
  // VESC プロトコル: Extended CAN ID = (CMD << 8) | VESC_ID
  //                 Data: 4 byte big-endian int32
  // ----------------------------------------------------------
  void send_vesc_rpm(uint8_t vesc_id, int32_t rpm) {
    if (can_socket_ < 0) return;

    struct can_frame frame;
    frame.can_id = (static_cast<uint32_t>(COMM_SET_RPM) << 8 | vesc_id) |
                   CAN_EFF_FLAG;  // Extended frame
    frame.can_dlc = 4;
    frame.data[0] = static_cast<uint8_t>((rpm >> 24) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((rpm >> 16) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((rpm >> 0) & 0xFF);

    if (write(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CAN 送信失敗 (ID=0x%02X): %s", vesc_id,
                           std::strerror(errno));
      can1_send_ok_ = false;
    } else {
      can1_send_ok_ = true;
    }
  }

  void publish_can1_status() {
    std_msgs::msg::Bool msg;
    msg.data = (can_socket_ >= 0) && can1_send_ok_;
    can1_status_pub_->publish(msg);
  }

  // ----------------------------------------------------------
  // メンバ変数
  // ----------------------------------------------------------
  int can_socket_;
  struct sockaddr_can addr_;
  struct ifreq ifr_;
  bool can1_send_ok_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr can_status_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can1_status_pub_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  int32_t target_erpm_[4]{0, 0, 0, 0};  // FL, FR, RL, RR
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescOmniNode>());
  rclcpp::shutdown();
  return 0;
}
