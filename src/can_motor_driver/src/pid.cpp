#include "can_motor_driver/pid.hpp"

#include <cassert>

PID::PID(double kp, double ki, double kd, Mode mode) {
  set_gains(kp, ki, kd);
  set_mode(mode);
  reset();
}

double PID::do_pid(double current_value) {
  assert(_dt > 0 && "PID error: dt is not set or is zero.");

  double error = _goal - current_value;
  double output = 0.0;

  if (_mode == Mode::POSITIONAL) {
    double p_term = _kp * error;
    _integral += error * _dt;
    double i_term = _ki * _integral;

    double d_term;
    if (_derivative_on_measurement_enabled) {
      d_term = -_kd * (current_value - _previous_value) / _dt;
    } else {
      d_term = _kd * (error - _previous_error) / _dt;
    }

    double pre_saturated_output = p_term + i_term + d_term;
    output = constrain(pre_saturated_output, _output_min, _output_max);

    if (_anti_windup_enabled && _ki != 0.0) {
      _integral += (output - pre_saturated_output) / _ki;
    }

  } else {  // Mode::VELOCITY
    double p_diff = _kp * (error - _previous_error);
    double i_diff = _ki * error * _dt;
    double d_diff;

    if (_derivative_on_measurement_enabled) {
      d_diff =
          -_kd * (current_value - 2 * _previous_value + _previous_value2) / _dt;
    } else {
      d_diff = _kd * (error - 2 * _previous_error + _previous_error2) / _dt;
    }

    double delta_output = p_diff + i_diff + d_diff;
    output = _last_output + delta_output;
    output = constrain(output, _output_min, _output_max);
  }

  _previous_error2 = _previous_error;
  _previous_error = error;
  _previous_value2 = _previous_value;
  _previous_value = current_value;
  _last_output = output;

  return output;
}

void PID::set_goal(double goal) { _goal = goal; }
void PID::set_dt(double dt_sec) {
  if (dt_sec > 0) _dt = dt_sec;
}
void PID::set_gains(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}
void PID::set_mode(Mode mode) {
  _mode = mode;
  reset();
}
void PID::set_output_limits(double min, double max) {
  _output_min = min;
  _output_max = max;
}
void PID::enable_anti_windup(bool enable) { _anti_windup_enabled = enable; }
void PID::enable_derivative_on_measurement(bool enable) {
  _derivative_on_measurement_enabled = enable;
}

void PID::reset() {
  _integral = 0.0;
  _previous_error = 0.0;
  _previous_error2 = 0.0;
  _previous_value = 0.0;
  _previous_value2 = 0.0;
  _last_output = 0.0;
}

double PID::constrain(double value, double min, double max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}
