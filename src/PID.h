#ifndef PID_H
#define PID_H

#include <iostream>
#include <list>
#include <numeric>
#include <math.h>
#include <algorithm>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

	/*
	* Others
	*/
	int n, window_size;
	double dp, di, dd, tol, best_err;
	list<double> cte_window;
	
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
  
  /*
  * Twiddle function
  */
  void Twiddle(double cte);
};

#endif /* PID_H */
