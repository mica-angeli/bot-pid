// Ported from a BSD-licensed C++ ROS library ( https://github.com/ros-controls/control_toolbox )
//  licensed by Willow Garage, Inc.

#include "pid.h"

float clamp(float val, float lo, float hi) {
  if(val > hi) {
    val = hi;
  }
  else if(val < lo) {
    val = lo;
  }

  return val;
}

void pid_set_gains(pid* pid, float p, float i, float d, float i_max, float i_min, bool antiwindup) {
  pid->p_gain_ = p;
  pid->i_gain_ = i;
  pid->d_gain_ = d;
  pid->i_max_ = i_max;
  pid->i_min_ = i_min;
  pid->antiwindup_ = antiwindup;
}

void pid_get_gains(const pid* pid, float* p, float* i, float* d, float* i_max, float* i_min, bool* antiwindup) {
  *p = pid->p_gain_;
  *i = pid->i_gain_;
  *d = pid->d_gain_;
  *i_max = pid->i_max_;
  *i_min = pid->i_min_;
  *antiwindup = pid->antiwindup_;
}

void pid_reset(pid* pid) {
  pid->p_error_last_ = 0.0;
  pid->p_error_ = 0.0;
  pid->i_error_ = 0.0;
  pid->d_error_ = 0.0;
  pid->cmd_ = 0.0;
}

float pid_compute_command(pid* pid, float error, float dt) {
  if(dt == 0.0f) {
    return 0.0f;
  }

  float error_dot = pid->d_error_;

  // Calculate the derivative error
  if(dt > 0.0f) {
    error_dot = (error - pid->p_error_last_) / dt;
    pid->p_error_last_ = error;
  }

  return pid_compute_command_with_error_dot(pid, error, error_dot, dt);
}

float pid_compute_command_with_error_dot(pid* pid, float error, float error_dot, float dt) {
  float p_term, d_term, i_term;
  pid->p_error_ = error; // this is error = target - state
  pid->d_error_ = error_dot;

  if(dt == 0.0f) {
    return 0.0f;
  }

  // Calculate proportional contribution to command
  p_term = pid->p_gain_ * pid->p_error_;

  // Calculate the integral of the position error
  pid->i_error_ += dt * pid->p_error_;

  if(pid->antiwindup_) {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    pid->i_error_ = clamp(pid->i_error_,
                          pid->i_min_ / fabsf(pid->i_gain_),
                          pid->i_max_ / fabsf(pid->i_gain_));
  }

  // Calculate integral contribution to command
  i_term = pid->i_gain_ * pid->i_error_;

  if(!pid->antiwindup_) {
    // Limit i_term so that the limit is meaningful in the output
    i_term = clamp(i_term, pid->i_min_, pid->i_max_);
  }

  // Calculate derivative contribution to command
  d_term = pid->d_gain_ * pid->d_error_;

  // Compute the command
  pid->cmd_ = p_term + i_term + d_term;

  return pid->cmd_;
}

void pid_print_values(const pid* pid) {
  printf("Current Values of PID Object:\n");
  printf(" P Gain: %f\n", pid->p_gain_);
  printf(" I Gain: %f\n", pid->i_gain_);
  printf(" D Gain: %f\n", pid->d_gain_);
  printf(" I_Max: %f\n", pid->i_max_);
  printf(" I_Min: %f\n", pid->i_min_);
  printf(" Antiwindup: %s\n", pid->antiwindup_? "true": "false");
  printf(" P_Error_Last: %f\n", pid->p_error_last_);
  printf(" P_Error: %f\n", pid->p_error_);
  printf(" I_Error: %f\n", pid->i_error_);
  printf(" D_Error: %f\n", pid->d_error_);
  printf(" Command: %f\n", pid->cmd_);
}
