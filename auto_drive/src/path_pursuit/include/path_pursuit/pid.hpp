#ifndef PID_HPP
#define PID_HPP

#include <string>
#include <iostream>

enum Torelance_type{
        None = 0,
        Stop,
        Keep
    };

class PID{
public:
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;

  float ctrl_period = 0.01;//sec
  float target = 0;

  bool max_limit_flag = false;
  float max_limit = 0;
  bool min_limit_flag = false;
  float min_limit = 0;
  bool accel_limit_flag = false;
  float accel_limit = 0;
  bool integral_limit_flag = false;
  float integral_limit = 0;

  bool torelance_judge_flag = false;
  bool torelance_flag = false;
  float torelance = 0;
  Torelance_type torelance_type = Keep;

  bool cmd_debug_flag = false;
  bool torelance_debug_flag = false;
  bool succeed_debug_flag = false;

  PID() = default;

  explicit PID(float Kp, float Ki, float Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
  }

  float pid_ctrl(float present_val){
    float cmd = 0;
    diff_updater(present_val);
    cmd = pid_logic();
    if (integral_limit_flag)
    {
      integral_limiter();
    }
    if (max_limit_flag)
    {
      cmd = max_limiter(cmd);
    }
    if (min_limit_flag)
    {
      cmd = min_limiter(cmd);
    }
    cmd = torelance_executor(cmd);
    cmd_buff_updater(cmd);
    if (accel_limit_flag)
    {
      cmd = accel_limiter();
    }
    if(cmd_debug_flag){
        std::cout << "cmd: " << cmd << std::endl;
    }
    return cmd;
  }

  float pid_ctrl(float present_val, float target){ // overload
    this->target = target;
    return pid_ctrl(present_val);
  }

  void integral_reset(){
    integral = 0;
  }

  void show_param(){
    std::cout << "Kp: " << Kp << std::endl;
    std::cout << "Ki: " << Ki << std::endl;
    std::cout << "Kd: " << Kd << std::endl;
    std::cout << "target: " << target << std::endl;
    std::cout << "max_limit: " << max_limit << std::endl;
    std::cout << "min_limit: " << min_limit << std::endl;
    std::cout << "accel_limit: " << accel_limit << std::endl;
    std::cout << "integral_limit: " << integral_limit << std::endl;
  }
private:
  float integral = 0;
  float diff[2] = {0, 0};
  float cmd_buff[2] = {0, 0};
  float succeed_count = 0;

  float torelance_executor(float val){
    float ret = val;
    if(abs(diff[1]) <= torelance){
        torelance_flag = true;
    }else{
        torelance_flag = false;
    }
    if(torelance_flag){
      integral = 0;
      switch (torelance_type)
      {
      case None:
          ret = val;
          break;
      case Stop:
          ret = 0;
          break;
      case Keep:
          ret = cmd_buff[1];
          break;
      default:
          ret = val;
          break;
      }
    }
    if(torelance_debug_flag){
            std::cout << "torelance executed"<< std::endl;
    }
    return ret;
  }

  // type safe sgn function
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  float max_limiter(float val){
    if(abs(val) > max_limit){
        val = sgn(val)*max_limit;
    }
    return val;
  }

  float min_limiter(float val){
    if(abs(val) < min_limit){
        val = sgn(val)*min_limit;
    }
    return val;
  }

  void cmd_buff_updater(float new_cmd){
    cmd_buff[0] = cmd_buff[1];
    cmd_buff[1] = new_cmd;
  }

  float accel_limiter(){
    if(abs(cmd_buff[1]-cmd_buff[0]) > accel_limit){
        cmd_buff[1] = cmd_buff[0] + sgn(cmd_buff[1]-cmd_buff[0])*accel_limit;
    }
    return cmd_buff[1];
  }

  void integral_limiter(){
      if(integral > integral_limit){
          integral = integral_limit;
      }
  }

  void diff_updater(float present_val){
      diff[0] = diff[1];
      diff[1] = target - present_val;
  }

  float pid_logic(){
      float manipulated_value;
      integral += (diff[1]+diff[0])/2.0*ctrl_period;//台形積分(扱いやすい)
      manipulated_value =this->Kp*diff[1]+this->Ki*integral+this->Kd*(diff[1]-diff[0])/ctrl_period;
      return manipulated_value;
  }
};
#endif // PID_HPP
