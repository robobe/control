#include <thread>
#include <cmath>
#include <gtest/gtest.h>
#include "control/pid.h"

bool almost_equal(double a, double b, double epsilon){
    auto value = std::abs(a - b);
    auto result = value < epsilon;
    return result;

}
TEST(PIDSuit, init){
    PID pid(1, 1, 1);
    pid.setSetpoint(1.0);
    pid.update(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    pid.update(0.0);
    auto output = pid.getOutput();
    EXPECT_TRUE(almost_equal(2.0, output, 0.1));
}