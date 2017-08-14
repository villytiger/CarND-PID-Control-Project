#include "pid.h"

#include <iostream>

void Pid::Init(double tau_p, double tau_d, double tau_i) {
  tau_p_ = tau_p;
  tau_d_ = tau_d;
  tau_i_ = tau_i;

  prev_cte_ = 0;
  total_cte_ = 0;

  prev_time_ = std::chrono::steady_clock::now();
}

double Pid::Update(double value, double cte) {
  auto current_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_diff = current_time - prev_time_;
  prev_time_ = current_time;

  double cte_diff = (cte - prev_cte_) / time_diff.count();
  prev_cte_ = cte;
  total_cte_ += cte * time_diff.count();

  return -tau_p_ * cte - tau_d_ * cte_diff - tau_i_ * total_cte_;
}
