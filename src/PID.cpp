#include "PID.h"

#include <iostream>
#include <numeric>
#include <sstream>

PID::PID(const std::string &name) : name_(name) {}

PID::~PID() {}

void PID::SetControlParams(double Kp, double Ki, double Kd) {
  control_params_[0] = Kp;
  control_params_[1] = Ki;
  control_params_[2] = Kd;
  twiddle_ = false;
  CreateCTEFile();
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
  CreateCTEFile();
}

void PID::CreateCTEFile() {
  std::stringstream  has_p;
  std::stringstream  has_i;
  std::stringstream  has_d;
  if (control_params_[0] > 0.0) {
    has_p << 'P' << control_params_[0] << "_";
  } else {
    has_p << "_";
  }
  if (control_params_[1] > 0.0) {
    has_p << 'I' << control_params_[1] << "_";
  } else {
    has_p << "_";
  }
  if (control_params_[2] > 0.0) {
    has_p << 'D' << control_params_[2] << "_";
  } else {
    has_p << "_";
  }

  std::stringstream file_name;
  file_name << name_ << has_p.str() << has_i.str() << has_d.str() << "_cte.csv";
  fs_.open(file_name.str().c_str(),
           std::fstream::out | std::fstream::trunc | std::ios::binary);
  fs_.setf(std::ios_base::fixed);
  fs_ << "P=" << control_params_[0] << "I=" << control_params_[1]
      << "D=" << control_params_[2] << std::endl;
  fs_ << "count, time, delta_t, total_error, cte, integral, derivative"
      << std::endl;
}

void PID::UpdateError(double cte) {
  i_error_ += cte;
  clock_t t = clock();
  if (first_update_) {
    prev_cte_ = cte;
    prev_time_ = t;
    first_update_ = false;
  }

  double dt = ((double)(t - prev_time_)) / CLOCKS_PER_SEC;
  if (dt > 0.0) {
    d_error_ = (cte - prev_cte_) / dt;
  }

  prev_cte_ = cte;
  prev_time_ = t;

  p_error_ = cte;
  CalculateError();

  fs_ << update_count_ << "," << t << "," << dt << "," << total_error_ << ","
      << p_error_ << "," << i_error_ << "," << d_error_ << std::endl;
  fs_.flush();
}

double PID::TotalError() { return total_error_; }

void PID::CalculateError() {
  double Kp = control_params_[0];
  double Ki = control_params_[1];
  double Kd = control_params_[2];

  total_error_ = (-1.0 * p_error_ * Kp) - (d_error_ * Kd) - (i_error_ * Ki);

  update_count_++;
  int sample_count = update_count_ % sample_size_;

  if (twiddle_ && sample_count == 0) {
    // Adjust the control parameters looking for an improvement.

    std::cout << "TwiddleParams" << std::endl;
    DebugPrint();
    std::cout << "---------------------" << std::endl;
    TwiddleParams();
    std::cout << "After Twiddle ---------------------" << std::endl;
    DebugPrint();
    std::cout << "---------------------" << std::endl << std::endl;
  }
  if (sample_count > 100) {
    double count = (sample_count - 100);
    total_sample_quadratic_error_ += (total_error_ * total_error_);
    sample_quadratic_error_ = total_sample_quadratic_error_ / count;
  }
}

void PID::TwiddleParams() {
  if (best_quadratic_error_ == std::numeric_limits<double>::infinity()) {
    best_quadratic_error_ = sample_quadratic_error_;
  }

  // No control params left to adjust.
  if (indexes.empty() && stage_ == Stage::INITIAL) {
    // No adjustment required.
    return;
  }

  // Now twiddle the control params to attempt to reduce the error.
  if (stage_ == Stage::INITIAL) {
    do {
      if (indexes.empty()) {
        std::cout << "No adjustment required" << std::endl;
        return;
      }
      current_parameter_ = indexes.front();
      indexes.pop_front();
    } while (dp_[current_parameter_] < 0.000001);
  }

  if (sample_quadratic_error_ < best_quadratic_error_) {
    // Improvement - widen the search slightly.
    std::cout << "+++++ :Improvement of "
              << best_quadratic_error_ - sample_quadratic_error_ << std::endl;

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
  // Rest the error for the next iteration.
  total_sample_quadratic_error_ = 0.0;
}

void PID::DebugPrint() {
  //  std::cout.setf(std::ios_base::fixed);
  // clang-format off
  std::cout << name_
            << ",uc:" << update_count_
            << "," << current_parameter_
            << ":" << StageToString(stage_)
            << "|pe:" << p_error_
            << ",ie:" << i_error_
            << ",de:" << d_error_
            << ",te:" << total_error_
            << "|tsqe:"<<  total_sample_quadratic_error_
            << ",sqe:" << sample_quadratic_error_
            << ",bqe:" << best_quadratic_error_
            << "|p:[" << control_params_[0]
            << "," << control_params_[1]
            << "," << control_params_[2]
            << "],dp_:[" << dp_[0]
            << "," << dp_[1]
            << "," << dp_[2]
            << "]" << std::endl;
  // clang-format on
}
