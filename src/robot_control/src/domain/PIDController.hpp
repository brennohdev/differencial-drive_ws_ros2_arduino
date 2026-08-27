#pragma once

namespace ddrobot::domain {

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd);

  double compute(double error, double delta_time);

  void reset();

private:
  double Kp_;
  double Ki_;
  double Kd_;
  double integral_;
  double previous_error_;
};

} // namespace ddrobot::domain