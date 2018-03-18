#ifndef PID_H
#define PID_H

#include <limits>
#include <list>
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

  void SetControlParamsWithTwiddle(double Kp, double Ki, double Kd,
                                        double delta_Kp, double delta_Ki,
                                        double delta_Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double best_error() {return best_quadratic_error_;}

  void DebugPrint();
private:
  enum class Stage {
    INITIAL =0,
    HIGHER = 1,
    LOWER = 2
  };

  std::string StageToString(Stage stage) {
    switch(stage) {
      case Stage::INITIAL: return "Initial";
      case Stage::HIGHER: return "Higher";
      case Stage::LOWER: return "Lower";
    }
  }

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
  // Need a sufficiently large sample to cover most of the track so we have sharp turns
  // captured in the average cost. The downside is it takes longer to run.
  int sample_size_ = 10000;
  int update_count_ = 0;
  double best_quadratic_error_ = std::numeric_limits<double>::infinity();
  double sample_quadratic_error_ = 0.0;
  double total_sample_quadratic_error_ = 0.0;
  std::vector<double> dp_ = {0.1, 0.0, 0.0};
  int current_parameter_ = 0;
  std::list<int> indexes = {0 , 1, 2};
  Stage stage_ = Stage::INITIAL;
};

#endif /* PID_H */
