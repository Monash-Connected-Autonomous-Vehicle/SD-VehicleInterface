// Copyright (c) 2026 Monash Connected Autonomous Vehicle (MCAV)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Monash Connected Autonomous Vehicle (MCAV) nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include "sd_vehicle_interface/sd_control.h"

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
