#pragma once
#include <chrono>
#include <iostream>

using std::cout;
constexpr auto steady_min = std::chrono::steady_clock::time_point::min();

class PID{
public:
    PID() = delete;
    PID(double kp, double ki, double kd);
    void update(double feedback);
    void setSetpoint(double value);
    double getOutput();
private:
    double kp_, ki_, kd_ = 0.0;
    double setpoint_ = 0.0;
    double error_ = 0.0;
    double last_error_ = 0.0;
    double dt_;
    double sum_error_ = 0;

    std::chrono::_V2::steady_clock::time_point last_time_ = steady_min;
};