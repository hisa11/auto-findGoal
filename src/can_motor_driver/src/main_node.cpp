/// main_node.cpp — メインコントローラーノード
///
/// 役割分担:
///   omni_controller   … /cmd_vel → ERPM 変換・ウォッチドッグ
///   c610_controller   … RPM フィードバック PID → 電流指令
///   turret_controller … YOLO 追従・起動シーケンス状態管理
///   main_node (本ファイル)
///     … 上記 3 コントローラーを統括
///     … 全 CAN 送受信 (UsbCan::send / recv) はここだけで行う
///     … DMMotor / C610 はフレーム生成ライブラリとして使用

#include <linux/can.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <future>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>

// ---- CAN ドライバ / フレーム生成ライブラリ ----
#include "can_motor_driver/c610.hpp"
#include "can_motor_driver/dm_motor.hpp"
#include "can_motor_driver/usb_can.hpp"

// ---- 各サブシステムのコントローラー (純粋ロジック) ----
#include "can_motor_driver/c610_controller.hpp"
#include "can_motor_driver/omni_controller.hpp"
#include "can_motor_driver/turret_controller.hpp"

// ============================================================
// 共通定数
// ============================================================
static constexpr int CONTROL_MS = 30;  // 制御周期 [ms]

// ============================================================
// メインコントローラーノード
// ============================================================
class MainControllerNode : public rclcpp::Node {
 public:
  MainControllerNode() : Node("main_controller_node") {
    // ---- コールバックグループ ----
    cb_sub_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_timer_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = cb_sub_;

    // ----------------------------------------------------------
    // ROSパラメータ
    // ----------------------------------------------------------
    use_dm_motor_ = declare_parameter<bool>("use_dm_motor", true);
    RCLCPP_INFO(get_logger(), "use_dm_motor = %s",
                use_dm_motor_ ? "true" : "false");

    // ----------------------------------------------------------
    // CAN バス初期化 (main_node だけが UsbCan を保持する)
    // ----------------------------------------------------------
    can0_ = std::make_shared<UsbCan>("can0");
    can1_ = std::make_shared<UsbCan>("can1");
    if (!can0_->is_open()) RCLCPP_ERROR(get_logger(), "CAN(can0) を開けません");
    if (!can1_->is_open())
      RCLCPP_WARN(get_logger(), "CAN(can1) 未接続 — 500ms ごとに再接続を試みます");

    // ----------------------------------------------------------
    // フレーム生成ライブラリ初期化
    //   DMMotor / C610 は UsbCan ポインタを保持しているが、
    //   send() を呼ぶのは main_node の send_all_can() のみ
    // ----------------------------------------------------------
    dm_pitch_ =
        std::make_shared<DMMotor>(can1_.get(), 0x01, DMMotor::Mode::POSITION);
    dm_yaw_ =
        std::make_shared<DMMotor>(can1_.get(), 0x02, DMMotor::Mode::VELOCITY);
    c610_drv_ = std::make_shared<C610>(can0_.get());

    // ----------------------------------------------------------
    // ロジックコントローラー初期化
    // ----------------------------------------------------------
    omni_ctrl_ = std::make_shared<OmniController>();

    // ---- C610 PID パラメータ (8軸分だが使用するのは先頭 4 軸)
    //   kp, ki, kd, output_min, output_max, PID::Mode
    //   Mode::VELOCITY = 速度型(増分型) — RM C610 で動作確認済み
    const std::array<C610MotorParam, c610_param::NUM_MOTORS> c610_params = {{
        // motor 1 (ID=1)
        {/*kp*/ 0.7, /*ki*/ 0.6, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 2 (ID=2)
        {/*kp*/ 0.6, /*ki*/ 0.0, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 3 (ID=3)
        {/*kp*/ 0.6, /*ki*/ 0.0, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 4 (ID=4)
        {/*kp*/ 0.6, /*ki*/ 0.0, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 5 (ID=5)
        {/*kp*/ 1.8, /*ki*/ 1.3, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 6 (ID=6)
        {/*kp*/ 1.7, /*ki*/ 1.3, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
        // motor 7 (ID=7)
        {/*kp*/ 0.9, /*ki*/ 0.6, /*kd*/ 0.0,
         /*out_min*/ -c610_param::MAX_POWER, /*out_max*/ c610_param::MAX_POWER,
         PID::Mode::VELOCITY},
    }};
    c610_ctrl_ = std::make_shared<C610Controller>(c610_params);
    turret_ctrl_ = std::make_shared<TurretController>();

    // pitch 初期目標をセット (ギア比込み)
    dm_pitch_->set_target_position(turret_ctrl_->get_pitch_rad(),
                                   turret_ctrl_->get_pitch_max_vel());
    dm_yaw_->set_target_velocity(0.0f);

    // use_dm_motor=false の場合は起動シーケンスをスキップ
    if (!use_dm_motor_) {
      turret_ctrl_->skip_startup();
      RCLCPP_INFO(get_logger(),
                  "DM モータ無効 — 起動シーケンスをスキップし C610 即起動");
    }

    // ----------------------------------------------------------
    // ROS サブスクライバー (Reentrant: 並列実行)
    // ----------------------------------------------------------
    goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
        "/yolo/goal_info", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          turret_ctrl_->update_goal(static_cast<float>(msg->x),
                                    static_cast<float>(msg->z),
                                    now().nanoseconds());
        },
        sub_opt);

    // モーターごとの目標 RPM トピックを購読 (/c610/motor1 ～ /c610/motor7)
    // rosbridge は Best Effort でパブリッシュするため QoS を合わせる
    auto c610_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    for (int id = 1; id <= c610_param::NUM_MOTORS; ++id) {
      std::string topic = "/c610/motor" + std::to_string(id) + "/target_rpm";
      auto sub = create_subscription<std_msgs::msg::Int32>(
          topic, c610_qos,
          [this, id](const std_msgs::msg::Int32::SharedPtr msg) {
            c610_ctrl_->set_target_rpm(id - 1, msg->data);
            RCLCPP_DEBUG(get_logger(), "[C610] モーター %d 目標 RPM → %d", id,
                        msg->data);
          },
          sub_opt);
      c610_target_subs_.push_back(sub);
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          omni_ctrl_->update_cmd(msg->linear.x, msg->linear.y, msg->angular.z,
                                 now().nanoseconds());
        },
        sub_opt);

    // ----------------------------------------------------------
    // ROS パブリッシャー
    // ----------------------------------------------------------
    can0_status_pub_ = create_publisher<std_msgs::msg::Bool>("/can_status/can0",
                                                             rclcpp::QoS(1));
    can1_status_pub_ = create_publisher<std_msgs::msg::Bool>("/can_status/can1",
                                                             rclcpp::QoS(1));

    // ----------------------------------------------------------
    // タイマー (MutuallyExclusive: CAN 送信を直列化)
    // ----------------------------------------------------------
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(CONTROL_MS),
        std::bind(&MainControllerNode::control_loop, this), cb_timer_);

    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MainControllerNode::publish_status, this), cb_timer_);

    last_control_time_ = now();

    RCLCPP_INFO(get_logger(), "MainControllerNode 起動 (制御周期=%dms)",
                CONTROL_MS);
  }

 private:
  // ==========================================================
  // 30ms 制御ループ
  // ==========================================================
  void control_loop() {
    // dt 計算
    auto t_now = now();
    double dt = (t_now - last_control_time_).seconds();
    last_control_time_ = t_now;
    if (dt <= 0.0 || dt > 1.0) dt = CONTROL_MS * 0.001;

    const int64_t now_ns = t_now.nanoseconds();

    // ----------------------------------------------------------
    // Step 1: CAN 受信
    //   can0 → C610 フィードバック (startup 完了後)
    //   can1 → DM モータ起動中は受信バッファをドレイン
    // ----------------------------------------------------------
    if (can0_->is_open() && turret_ctrl_->startup_done()) {
      c610_drv_->param_update();
      for (int i = 0; i < c610_param::NUM_MOTORS; ++i) {
        c610_ctrl_->feed_rpm(i, c610_drv_->get_rpm(i + 1));
      }
    }
    if (can1_->is_open() && use_dm_motor_ && !turret_ctrl_->startup_done()) {
      can1_->drain();
    }

    // ----------------------------------------------------------
    // Step 2: 並列演算
    //   [A] C610 PID 4軸
    //   [B] オムニウォッチドッグ
    //   [C] 砲塔タイムアウト判定
    // ----------------------------------------------------------
    auto fut_c610 = std::async(std::launch::async,
                               [this, dt]() { c610_ctrl_->compute(dt); });
    auto fut_omni = std::async(std::launch::async, [this, now_ns]() {
      omni_ctrl_->check_watchdog(now_ns);
    });
    auto fut_turret = std::async(std::launch::async, [this, now_ns]() {
      if (turret_ctrl_->startup_done()) turret_ctrl_->step(now_ns);
    });

    fut_c610.wait();
    fut_omni.wait();
    fut_turret.wait();

    // ----------------------------------------------------------
    // Step 3: コントローラー出力をフレーム生成ライブラリに反映
    // ----------------------------------------------------------
    apply_controller_outputs();

    // ----------------------------------------------------------
    // Step 4: 全 CAN 送信 (send_all_can のみが UsbCan::send を呼ぶ)
    // ----------------------------------------------------------
    send_all_can();
  }

  // ==========================================================
  // コントローラー出力をフレーム生成ライブラリに適用
  // ==========================================================
  void apply_controller_outputs() {
    // --- 砲塔: 起動シーケンス ---
    if (!turret_ctrl_->startup_done()) {
      StartupCmd cmd = turret_ctrl_->tick_startup();
      switch (cmd) {
        case StartupCmd::PITCH_SET_MODE:
          dm_pitch_->set_control_mode(DM_POSITION_MODE);
          RCLCPP_INFO(get_logger(), "Turret: pitch POSITION_MODE 設定");
          break;
        case StartupCmd::PITCH_ENABLE:
          dm_pitch_->enable();
          RCLCPP_INFO(get_logger(), "Turret: pitch enable");
          break;
        case StartupCmd::YAW_SET_MODE:
          dm_yaw_->set_control_mode(DM_VELOCITY_MODE);
          RCLCPP_INFO(get_logger(), "Turret: yaw VELOCITY_MODE 設定");
          break;
        case StartupCmd::YAW_ENABLE:
          dm_yaw_->enable();
          RCLCPP_INFO(get_logger(), "Turret: yaw enable");
          break;
        default:
          break;
      }
      if (turret_ctrl_->startup_done())
        RCLCPP_INFO(get_logger(), "起動完了。YOLO 追従開始。");
      return;  // 起動中はここで終了 (CAN 送信もしない)
    }

    // --- 砲塔: 通常制御 ---
    dm_yaw_->set_target_velocity(turret_ctrl_->get_yaw_velocity());

    // --- C610: 電流指令をドライバにセット (target=0 でも PID 出力を渡す) ---
    for (int i = 0; i < c610_param::NUM_MOTORS; ++i) {
      c610_drv_->set_power(i + 1, c610_ctrl_->get_power(i));
    }
    // 診断ログ: 約1秒ごとに全軸のtarget/power/actual_rpmを出力
    if (++diag_count_ % 33 == 0) {
      RCLCPP_INFO(get_logger(),
                  "[C610 CAN] 0x200(motor1-4):%s  0x1FF(motor5-8):%s",
                  c610_drv_->get_last_ok_200() ? "OK" : "FAIL",
                  c610_drv_->get_last_ok_1ff() ? "OK" : "FAIL");
      for (int i = 0; i < c610_param::NUM_MOTORS; ++i) {
        RCLCPP_INFO(get_logger(),
                    "[C610 diag] motor%d  target=%d  power=%d  actual_rpm=%d",
                    i + 1, c610_ctrl_->get_target_rpm(i),
                    c610_ctrl_->get_power(i),
                    static_cast<int>(c610_drv_->get_rpm(i + 1)));
      }
    }
    // ※ stop() は呼ばない — target=0 時も PID が能動ブレーキを担当
  }

  // ==========================================================
  // ★ CAN 送信統合関数 — UsbCan::send を呼ぶのはここだけ
  // ==========================================================
  void send_all_can() {
    bool ok0 = true, ok1 = true;

    // can0: C610 ESC のみ
    if (can0_->is_open() && turret_ctrl_->startup_done()) {
      ok0 &= c610_drv_->send_message();
    }

    // can1: DM モータ + VESC 4輪
    if (can1_->is_open()) {
      if (use_dm_motor_) {
        ok1 &= dm_pitch_->send();
        ok1 &= dm_yaw_->send();
      }
      const uint8_t ids[4] = {omni_param::ID_FL, omni_param::ID_FR,
                              omni_param::ID_RL, omni_param::ID_RR};
      for (int i = 0; i < 4; ++i)
        ok1 &= send_vesc_frame(ids[i], omni_ctrl_->get_erpm(i));
    }

    can0_send_ok_.store(ok0);
    can1_send_ok_.store(ok1);
  }

  /// VESC CAN 拡張フレーム構築・送信
  bool send_vesc_frame(uint8_t vesc_id, int32_t erpm) {
    struct can_frame frame{};
    frame.can_id =
        (static_cast<uint32_t>(omni_param::COMM_SET_RPM) << 8 | vesc_id) |
        CAN_EFF_FLAG;
    frame.can_dlc = 4;
    frame.data[0] = static_cast<uint8_t>((erpm >> 24) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((erpm >> 16) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((erpm >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((erpm >> 0) & 0xFF);
    return can1_->send(frame);
  }

  // ==========================================================
  // CAN ステータスパブリッシュ + can1 自動再接続 (500ms タイマー)
  // ==========================================================
  void publish_status() {
    // can1 が未オープンなら再接続を試みる
    if (!can1_->is_open()) {
      if (can1_->try_open()) {
        RCLCPP_INFO(get_logger(), "CAN(can1) 再接続成功 — VESC 有効化");
      }
    }

    std_msgs::msg::Bool m;
    m.data = can0_->is_open() && can0_send_ok_.load();
    can0_status_pub_->publish(m);
    m.data = can1_->is_open() && can1_send_ok_.load();
    can1_status_pub_->publish(m);

    // VESC 診断ログ (can1 が開いている時のみ)
    if (can1_->is_open()) {
      const char* wheels[4] = {"FL", "FR", "RL", "RR"};
      for (int i = 0; i < 4; ++i) {
        RCLCPP_DEBUG(get_logger(), "[VESC diag] %s  erpm=%d",
                     wheels[i], omni_ctrl_->get_erpm(i));
      }
    }
  }

  // ==========================================================
  // メンバ変数
  // ==========================================================

  // パラメータ
  bool use_dm_motor_{true};

  // CAN バス (main_node だけが保持)
  std::shared_ptr<UsbCan> can0_, can1_;

  // フレーム生成ライブラリ
  std::shared_ptr<DMMotor> dm_pitch_, dm_yaw_;
  std::shared_ptr<C610> c610_drv_;

  // ロジックコントローラー
  std::shared_ptr<OmniController> omni_ctrl_;
  std::shared_ptr<C610Controller> c610_ctrl_;
  std::shared_ptr<TurretController> turret_ctrl_;

  // CAN 送信成否
  std::atomic<bool> can0_send_ok_{false}, can1_send_ok_{false};

  // 制御時間
  rclcpp::Time last_control_time_;

  // コールバックグループ
  rclcpp::CallbackGroup::SharedPtr cb_sub_, cb_timer_;

  // サブスクライバー
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> c610_target_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // パブリッシャー
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can0_status_pub_,
      can1_status_pub_;

  // 診断カウンター
  int diag_count_{0};

  // タイマー
  rclcpp::TimerBase::SharedPtr control_timer_, status_timer_;
};

// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{},
                                                4 /* スレッド数 */);
  auto node = std::make_shared<MainControllerNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
