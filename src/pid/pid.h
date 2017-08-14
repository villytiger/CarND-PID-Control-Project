#ifndef PID_H
#define PID_H

#include <chrono>

class Pid {
 public:
  void Init(double tau_p, double tau_d, double tau_i);

  double Update(double value, double cte);

 private:
  double tau_p_;
  double tau_d_;
  double tau_i_;

  double prev_cte_ = 0;
  double total_cte_ = 0;

  std::chrono::time_point<std::chrono::steady_clock> prev_time_;
};

#endif /* PID_H */
