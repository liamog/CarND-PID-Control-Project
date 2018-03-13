#ifndef PID_H
#define PID_H

#include <limits>
#include <string>
#include <vector>

class PID {

 public:
  /*
  * Constructor
  */
  PID(const std::string &name);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void SetControlParams(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double best_error() {return best_quadratic_error_;}
  void EnableTwiddle() {twiddle_ = true;}

  void DebugPrint();
private:
  void CalculateError();
  void TwiddleParams();

  std::string name_;
  /*
  * Errors
  */
  double p_error_ = 0.0;
  double i_error_ = 0.0;
  double d_error_ = 0.0;
  // Parameters
  std::vector<double> control_params_ = { 0.0, 0.0, 0.0 };

  double current_error_ = 0.0;

  bool first_update_ = true;
  double prev_cte_ = 0.0;

  // Twiddle params
  bool twiddle_ = false;
  int sample_size_ = 1000;
  int update_count_ = 0;
  double best_quadratic_error_ = std::numeric_limits<double>::infinity();
  double sample_quadratic_error_ = 0.0;
  double tolerance_threshold_ = 0.0001;
  double current_tolerance_ = std::numeric_limits<double>::infinity();
  std::vector<double> dp_ = {0.1, 0.0, 0.0};
  int current_parameter_ = 0;
  bool first_run_ = true;
  std::list<int> indexes = {0 , 1, 2}
};

#endif /* PID_H */
