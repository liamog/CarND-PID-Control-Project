#include "PID.h"

#include <iostream>
#include <numeric>

PID::PID(const std::string &name) : name_(name) {}

PID::~PID() {}

void PID::SetControlParams(double Kp, double Ki, double Kd) {
  control_params_[0] = Kp;
  control_params_[1] = Ki;
  control_params_[2] = Kd;
}

void PID::UpdateError(double cte) {
  i_error_ += cte;

  if (first_update_) {
    prev_cte_ = cte;
    first_update_ = false;
  }
  d_error_ = cte - prev_cte_;
  prev_cte_ = cte;

  p_error_ = cte;
  CalculateError();
}

double PID::TotalError() { return current_error_; }

void PID::CalculateError() {
  double Kp = control_params_[0];
  double Ki = control_params_[1];
  double Kd = control_params_[2];

  current_error_ = (-1.0 * p_error_ * Kp) - (d_error_ * Kd) - (i_error_ * Ki);

  update_count_++;
  int sample_count = update_count_ % sample_size_;

  sample_quadratic_error_ += current_error_ * current_error_;
  sample_quadratic_error_ /= sample_count + 1;

  if (twiddle_ && sample_count == 0) {
    // Adjust the control parameters looking for an improvement.
    TwiddleParams();
  }
}

void PID::TwiddleParams() {
  current_tolerance_ =
      std::accumulate(dp_.begin(), dp_.end(), 0.0);
  // This function will be called once or twice for each current_parameter_
  // iteration. Depending on if there is an improvement.
  DebugPrint();

  std::cout << "TwiddleParams" << std::endl;
  if (current_tolerance_ < tolerance_threshold_) {
    // No adjustment required.
    std::cout << "No adjustment required" << std::endl;
    return;
  }

  if (sample_quadratic_error_ < best_error_) {
    // Improvement
    std::cout << current_parameter_ << ":Improvement " << current_parameter_
              << std::endl;
    best_error_ = sample_quadratic_error_;
    dp_[current_parameter_] *= 1.1;

    first_run_ = true;
    current_parameter_ = (current_parameter_ + 1) % control_params_.size();
    control_params_[current_parameter_] += dp_[current_parameter_];
  } else {  // No improvement
    if (first_run_) {
      std::cout << current_parameter_ << ":first_run No Improvement"
                << dp_[current_parameter_] << std::endl;

      control_params_[current_parameter_] -= 2.0 * dp_[current_parameter_];
      first_run_ = false;
    } else {
      dp_[current_parameter_] *= 0.9;

      std::cout << current_parameter_ << ":second run No Improvement"
                << std::endl;
      current_parameter_ = (current_parameter_ + 1) % control_params_.size();
      first_run_ = true;
      control_params_[current_parameter_] += dp_[current_parameter_];
    }
  }
  // Rest the error for the next iteration.
  sample_quadratic_error_ = 0.0;
}
void PID::DebugPrint() {
  std::cout.setf(std::ios_base::fixed);
  // clang-format off
  std::cout << name_
            << ",uc:" << update_count_
            << ",pe:" << p_error_
            << ",ie:" << i_error_
            << ",de:" << d_error_
            << ",te:" << current_error_
            << ",ci:" << current_parameter_
            << ",ct:" << current_tolerance_
            << ",p:[" << control_params_[0]
            << "," << control_params_[1]
            << "," << control_params_[2]
            << "],dp_:[" << dp_[0]
            << "," << dp_[1]
            << "," << dp_[2]
            << "]" << std::endl;
  // clang-format on
}
