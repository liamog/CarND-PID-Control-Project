#include "PID.h"

#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
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
  // std::cout << "cte: " << cte
  //           << " d_error_: " << d_error_
  //           << " p_error_: " << p_error_
  //           << " i_error_: " << i_error_
  //           << " prev_cte_: " << prev_cte_
  //           << std::endl;
}

double PID::TotalError() {
  return (-1.0 * p_error_ * Kp_) - (d_error_ * Kd_) - (i_error_ * Ki_);
}
