#ifndef PID_H
#define PID_H

class PID {

 public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  /*
  * Errors
  */
  double p_error_ = 0.0;
  double i_error_ = 0.0;
  double d_error_ = 0.0;

  /*
  * Coefficients
  */
  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  bool first_update_ = true;
  double prev_cte_ = 0.0;
};

#endif /* PID_H */
