#include "PID.h"
#include <numeric>
#include <limits>
#include <math.h>
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */

   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;

   p_error = 0;
   i_error = 0;
   d_error = 0;


   sum_of_squared_cte = 0;
   best_error = std::numeric_limits<double>::max();
   error = 0;

   dp[0] = dp[1] = dp[2] = 1.0;

   params[0] = Kp_;
   params[1] = Kd_;
   params[2] = Ki_;

   best_params[0] = Kp_;
   best_params[1] = Kd_;
   best_params[2] = Ki_;

   current_p = 0;
   dp_coefficient = 1;
   n = 0;

   count_iterations = 0;

}

void PID::Init(double Kp_, double Kd_, double Ki_, double dp_p, double dp_d, double dp_i) {
  Init(Kp_, Kd_, Ki_);
  dp[0] = dp_p;
  dp[1] = dp_d;
  dp[2] = dp_i;
}

void PID::SetBestValues(double best_error_, double best_Kp, double best_Kd, double best_Ki) {
  best_error = best_error_;
  best_params[0] = best_Kp;
  best_params[1] = best_Kd;
  best_params[2] = best_Ki;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
   double old_cte = p_error;

   p_error = cte;
   i_error += cte;
   d_error = cte - old_cte;

   // if(n >= MAX_N) {
     sum_of_squared_cte += pow(cte, 2);
     error = sum_of_squared_cte  / (2*MAX_N);

     // if (n % 50 == 0)
     // std::cout << "n: " << n
     //          << " sum_of_squared_cte: " << sum_of_squared_cte
     //           << " error: " << error << std::endl;
   // }
   n++;
   // std::cout << "n: " << n << " p_error: " << p_error << " i_error: "
   //           << i_error << " d_error: " << d_error << std::endl;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */

  return -Kp*p_error - Kd*d_error - Ki*i_error;
}

int PID::Twiddle() {
  if (dp[0] + dp[1] + dp[2]  <= TOLERANCE) {
    return 0;
  }

  if (dp_coefficient == 0) {
    dp_coefficient = 1;
    params[current_p] += dp[current_p];
    return -1;
  }

  if (n == MAX_N * 2) {
    std::cout << "error : " << error << std::endl;
    std::cout << "params[0]: " << params[0]
              << " params[1]: " << params[1]
              << " params[2]: " << params[2] << std::endl;

    PrintBestParams();

    if (dp_coefficient == 1) {
      if (error < best_error) {
        best_error = error ;
        best_params[0] = params[0];
        best_params[1] = params[1];
        best_params[2] = params[2];
        PrintBestParams();

        dp[current_p] *= 1.1;
        dp_coefficient = 0;
        current_p = (current_p + 1) % 3;

        return Twiddle();
      } else {
        dp_coefficient = -2;
        params[current_p] -= 2 * dp[current_p];

        return -1;
      }
    } else if (dp_coefficient == -2) {
      if (error < best_error) {
        best_error = error;
        best_params[0] = params[0];
        best_params[1] = params[1];
        best_params[2] = params[2];
        PrintBestParams();

        dp[current_p] *= 1.1;
      } else {
        params[current_p] += dp[current_p];
        dp[current_p] *= 0.9;
      }
      dp_coefficient = 0;
      current_p = (current_p + 1) % 3;

      return Twiddle();
    }
  }
  return -1;
}


bool PID::ShouldReset() {
  return n == MAX_N * 2 && (dp[0] + dp[1] + dp[2] > TOLERANCE);
}

void PID::PrintParams() {
  std::cout << "Kp: " << Kp << " Kd: " << Kd << " Ki: " << Ki << std::endl;
}

void PID::PrintBestParams() {
  std::cout << "best_error: " << best_error << std::endl;
  std::cout << "best_params[0]: " << best_params[0]
            << " best_params[1]: " << best_params[1]
            << " best_params[2]: " << best_params[2] << std::endl;

}

void PID::Reset() {
  std::cout << "reseting iteration vals" << std::endl;
  n = 0;
  sum_of_squared_cte = 0;
  p_error = 0;
  d_error = 0;
  i_error = 0;
  Kp = params[0];
  Kd = params[1];
  Ki = params[2];
  std::cout << "count_iterations: " << count_iterations + 1
            << " dp[0]: " << dp[0]
            << " dp[1]: " << dp[1]
            << " dp[2]: " << dp[2]
            << " sum: " << dp[0] + dp[1] + dp[2] << std::endl;

  // std::cout << "\t\t "
  //           << " params[0]: " << params[0]
  //           << " params[1]: " << params[1]
  //           << " params[2]: " << params[2] << std::endl;

  count_iterations += 1;

}
