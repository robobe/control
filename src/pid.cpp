#include "control/pid.h"

PID::PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd)
{
}

/// @brief calc PID output
/// @return 
double PID::getOutput(){
    double pTerm = error_ * kp_;
    double i_part = error_ * ki_ * dt_;
    double dTerm = 0;
    if (dt_ > 0){
        double d_error = (error_ - last_error_) / dt_;
        dTerm = d_error * kd_;
    }
    sum_error_ += i_part;
    //TODO: wind
    double iTerm = sum_error_;
    double output = pTerm + iTerm + dTerm;
    return output;
}

void PID::update(double feedback){
    auto current = std::chrono::steady_clock::now();
    if (last_time_ != steady_min){
        std::chrono::duration<double> sec = current - last_time_;
        dt_ = sec.count();
    }
    last_time_ = current;
    
    
    error_ = setpoint_ - feedback;
    last_error_ = error_;
}

void PID::setSetpoint(double value){
    setpoint_ = value;
}
