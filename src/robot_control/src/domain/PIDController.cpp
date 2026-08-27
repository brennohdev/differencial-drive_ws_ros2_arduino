#include "PIDController.hpp"

namespace ddrobot::domain {

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), integral_(0.0), previous_error_(0.0) {}

double PIDController::compute(double error, double delta_time) {
  integral_ += error * delta_time;
  double derivative = (error - previous_error_) / delta_time;
  previous_error_ = error;

  return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
}

void PIDController::reset() {
  integral_ = 0.0;
  previous_error_ = 0.0;
}
} // namespace ddrobot::domain