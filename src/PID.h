#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

 private:
  // Limit I scope
  int i;
  double *window;
  double total_cte;
  
  // Backprop
  double epoch_absolute_error;
  double epoch_squared_error;
  int epoch_count;
  double prev_rmse;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  void Backprop();
};

#endif  // PID_H