#pragma once
/// pid.hpp — 位置型 / 速度型(増分型) PID コントローラー
///   マイコン版 (mbed) で RM C610
///   モータへの有効性が確認されたライブラリの移植版。

class PID {
 public:
  enum class Mode {
    POSITIONAL,  // 位置型PID
    VELOCITY     // 速度型(増分型)PID
  };

  PID(double kp, double ki, double kd, Mode mode = Mode::POSITIONAL);
  double do_pid(double current_value);
  void set_goal(double goal);
  double get_goal() const { return _goal; }
  void set_dt(double dt_sec);
  void set_gains(double kp, double ki, double kd);
  void set_mode(Mode mode);
  void set_output_limits(double min, double max);
  void enable_anti_windup(bool enable);
  void enable_derivative_on_measurement(bool enable);
  void reset();

 private:
  double _kp, _ki, _kd;
  double _goal = 0.0;
  double _dt = 0.01;
  Mode _mode = Mode::POSITIONAL;
  double _integral = 0.0;
  double _previous_error = 0.0;
  double _previous_error2 = 0.0;
  double _previous_value = 0.0;
  double _previous_value2 = 0.0;
  double _last_output = 0.0;
  double _output_min = -1.0;
  double _output_max = 1.0;
  bool _anti_windup_enabled = true;
  bool _derivative_on_measurement_enabled = false;
  double constrain(double value, double min, double max);
};
