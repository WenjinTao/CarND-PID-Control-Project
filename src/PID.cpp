#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp; // Proportional coefficient
  this->Ki = Ki; // Integral coefficient
  this->Kd = Kd; // Differential 

  p_error = 0.0; // cte
  i_error = 0.0; // integral cte
  d_error = 0.0; // differential cte
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; // current_cte - previous_cte
  p_error = cte; // save the current_cte as previous_cte after calculating the d_error
  i_error += cte; // accumulating
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error; // Total weighted error
}

