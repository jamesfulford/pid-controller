#include "PID.h"

#include <iostream>
#include <string>
#include <cmath>

using std::string;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Integral calculations
  window = (double*) calloc(35, sizeof(window[0]));
  i = 35 - 1;
  total_cte = 0.0;

  // Backprop
  epoch_absolute_error = 0.0;
  epoch_squared_error = 0.0;
  epoch_count = 0;
  prev_rmse = 0.0;
  
  // Core error calculations
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error);
  p_error = cte;
  i_error = total_cte;
  
  // Limit integral to limited window of past measurements
  i = (i + 1) % 35;
  total_cte = total_cte - window[i] + cte;
  window[i] = cte;
  
  // Prepare for backprop
  epoch_absolute_error += std::fabs(cte);
  epoch_squared_error += std::pow(cte, 2.0);
  epoch_count += 1;
  
  if (epoch_count % 50 == 0) {
    Backprop();
  }
}

double PID::TotalError() {   
  return (
    (Kp * p_error)
    + (Ki * i_error)
    + (Kd * d_error)
  );
}

void PID::Backprop() {
  double rmse = std::sqrt(epoch_squared_error / epoch_count);
  double delta_error = prev_rmse - rmse;
  
  Kp -= Kp * (-p_error) * delta_error * 0.01;
  Ki -= Ki * (-epoch_absolute_error) * delta_error * 0.01;
  Kd -= Kd * (-d_error) * delta_error * 0.01;

  // Store rmse for next backprop
  prev_rmse = rmse;
  
  // Reset backprop accumulators
  epoch_squared_error = 0.0;
  epoch_absolute_error = 0.0;
  epoch_count = 0;
  
  std::cout << "p: " << Kp << " i: " << Ki << " d: " << Kd << " rmse: " << rmse << std::endl;
}
