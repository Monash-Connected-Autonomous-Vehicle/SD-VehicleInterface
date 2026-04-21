#include <gtest/gtest.h>

#include "sd_control.h"

TEST(SDControlTest, SteerRequestSaturatesToConfiguredLimits)
{
    EXPECT_EQ(100, speedcontroller::CalculateSteerRequest(1.0));
    EXPECT_EQ(-100, speedcontroller::CalculateSteerRequest(-1.0));
}

TEST(SDControlTest, SteerRequestZeroAngleReturnsZero)
{
    EXPECT_EQ(0, speedcontroller::CalculateSteerRequest(0.0));
}

TEST(SDControlTest, TwizyTorqueRequestRemainsWithinBounds)
{
    int proportional = 0;
    int integral = 0;
    int derivative = 0;
    int feedforward = 0;

    const int8_t torque = speedcontroller::CalculateTorqueRequestTwizy(
        2.0, 1.0, proportional, integral, derivative, feedforward);

    EXPECT_GE(torque, MIN_TORQUE_TWIZY);
    EXPECT_LE(torque, MAX_TORQUE_TWIZY);
}

TEST(SDControlTest, Env200StandstillUsesBrakeHoldTorque)
{
    int proportional = 0;
    int integral = 0;
    int derivative = 0;
    int feedforward = 0;

    const int8_t torque = speedcontroller::CalculateTorqueRequestEnv200(
        0.0, 0.0, proportional, integral, derivative, feedforward);

    EXPECT_EQ(BRAKE_HOLD_TORQUE_ENV200, torque);
    EXPECT_EQ(0, proportional);
    EXPECT_EQ(0, integral);
    EXPECT_EQ(0, derivative);
}
