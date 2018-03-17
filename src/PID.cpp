#include "PID.h"

#include <iostream>
#include <numeric>

PID::PID(const std::string &name) : name_(name) {}

PID::~PID() {}

void PID::SetControlParams(double Kp, double Ki, double Kd) {
  control_params_[0] = Kp;
  control_params_[1] = Ki;
  control_params_[2] = Kd;
  twiddle_ = false;
}

void PID::SetControlParamsWithTwiddle(double Kp, double Ki, double Kd,
                           double delta_Kp, double delta_Ki,
                           double delta_Kd) {
  control_params_[0] = Kp;
  control_params_[1] = Ki;
  control_params_[2] = Kd;

  dp_[0] = delta_Kp;
  dp_[1] = delta_Ki;
  dp_[2] = delta_Kd;
  twiddle_ = true;
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
  std::cout << ">>>>>>>> TwiddleParams - start" << std::endl;
  DebugPrint();
  std::cout << "---------------------" << std::endl;

  if (best_quadratic_error_ == std::numeric_limits<double>::infinity()) {
    best_quadratic_error_ = sample_quadratic_error_;
  }

  // No control params left to adjust.
  if (indexes.empty() && stage_ == Stage::INITIAL) {
    // No adjustment required.
    std::cout << "No adjustment required" << std::endl;
    return;
  }

  // Now twiddle the control params to attempt to reduce the error.
  if (stage_ == Stage::INITIAL) {
    current_parameter_ = indexes.front();
    indexes.pop_front();
  }

  if (sample_quadratic_error_ < best_quadratic_error_) {
    // Improvement - widen the search slightly.
    std::cout << "+++++++++ :Improvement " << std::endl;
    best_quadratic_error_ = sample_quadratic_error_;
    dp_[current_parameter_] *= 1.1;
    switch (stage_) {
      case Stage::INITIAL:
        control_params_[current_parameter_] += dp_[current_parameter_];
        stage_ = Stage::HIGHER;
        break;
      case Stage::HIGHER:
        stage_ = Stage::INITIAL;
        indexes.push_back(current_parameter_);
        TwiddleParams();
        break;
      case Stage::LOWER:
        stage_ = Stage::INITIAL;
        indexes.push_back(current_parameter_);
        TwiddleParams();
        break;
    }
  } else {  // No improvement
    std::cout << "-------- No Improvement" << std::endl;
    switch (stage_) {
      case Stage::INITIAL:
        control_params_[current_parameter_] += dp_[current_parameter_];
        stage_ = Stage::HIGHER;
        break;
      case Stage::HIGHER:
        control_params_[current_parameter_] -= 2 * dp_[current_parameter_];
        stage_ = Stage::LOWER;
        break;
      case Stage::LOWER:
        // Return to original value
        control_params_[current_parameter_] += dp_[current_parameter_];
        stage_ = Stage::INITIAL;

        // Narrow the search range.
        dp_[current_parameter_] *= .9;

        // Move to next param.
        indexes.push_back(current_parameter_);
        TwiddleParams();

        break;
    }
  }

  std::cout << "<<<<<<<< TwiddleParams - End" << std::endl;
  DebugPrint();
  std::cout << "---------------------" << std::endl;

  // Rest the error for the next iteration.

  sample_quadratic_error_ = 0.0;
}

void PID::DebugPrint() {
  std::cout.setf(std::ios_base::fixed);
  // clang-format off
  std::cout << name_
            << ",uc:" << update_count_
            << "," << current_parameter_
            << ":" << StageToString(stage_)
//             << ",pe:" << p_error_
//             << ",ie:" << i_error_
//             << ",de:" << d_error_
             << ",te:" << current_error_
            << ",sqe:" << sample_quadratic_error_
            << ",bqe:" << best_quadratic_error_
            << ",p:[" << control_params_[0]
            << "," << control_params_[1]
            << "," << control_params_[2]
            << "],dp_:[" << dp_[0]
            << "," << dp_[1]
            << "," << dp_[2]
            << "]" << std::endl;
  // clang-format on
}
