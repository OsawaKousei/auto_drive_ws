#ifndef PID_HPP
#define PID_HPP

#include <string>
#include <iostream>

class PID{

public:
  PID(double kp, double ki, double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->pre_error = 0;
    this->integral = 0;
    this->dt = 0.1;
    this->max = 2.0;
  }

  double pid_ctrl(double current_val, double target_val){
    double error = target_val - current_val;
    // std::cout << "error: " << error << std::endl;
    integral += error * dt;
    double diff = (error - pre_error);
    double cmd_val = kp * error + ki * integral + kd * diff;
    if(abs(cmd_val) > max){
      cmd_val = max * sgn(cmd_val);
    }
    pre_error = error;
    // std::cout << "cmd_val: " << cmd_val << std::endl;
    return cmd_val;
  }

  std::tuple<double, double, double> get_gains(){
    return std::make_tuple(kp, ki, kd);
  }

private:
  double kp;
  double ki;
  double kd;
  double pre_error;
  double integral;
  double dt;
  double max;

  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }
};
#endif // PID_HPP
