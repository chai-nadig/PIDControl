#ifndef PID_H
#define PID_H

#define MAX_N 1000
#define TWIDDLE false
#define TOLERANCE 0.001

#include <vector>

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

  void Init(double Kp_, double Kd_, double Ki_, double dp_p, double dp_d, double dp_i);

  void SetBestValues(double best_error_, double best_Kp, double best_Kd, double best_Ki);

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
   * Twiddle
   */
  int Twiddle();


  /**
   * Should reset
   */
  bool ShouldReset();

  /**
   *
   */
  void PrintParams();

  void PrintBestParams();


  void Reset();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /**
   * Cross Track Errors
   */
  double sum_of_squared_cte;
  double best_error;
  double error;

   /**
   * Twiddle params
   */
  double params[3];
  double best_params[3];
  double dp[3];

  int current_p;
  int dp_coefficient;
  int n;

  int count_iterations;

};

#endif  // PID_H
