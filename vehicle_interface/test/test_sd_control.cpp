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

// ============================================================================
// Test CalculateSteerRequest - Pure function, no static state
// ============================================================================

namespace speedcontroller
{

// Test fixture for CalculateSteerRequest tests
class TestCalculateSteerRequest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test: Normal inputs for CalculateSteerRequest
TEST_F(TestCalculateSteerRequest, NormalInputs) {
  // Input: 0.0 rad -> expected output: 0
  int8_t result = CalculateSteerRequest(0.0);
  EXPECT_EQ(result, 0);

  // Input: 0.5 rad (middle of range) -> expected output: ~71
  result = CalculateSteerRequest(0.5);
  EXPECT_EQ(result, 71);  // (0.5 / 0.69813) * 100 ≈ 71

  // Input: MAX_STEER_ANG (0.69813 rad) -> expected output: 100
  result = CalculateSteerRequest(MAX_STEER_ANG);
  EXPECT_EQ(result, 100);

  // Input: MIN_STEER_ANG (-0.69813 rad) -> expected output: -100
  result = CalculateSteerRequest(MIN_STEER_ANG);
  EXPECT_EQ(result, -100);
}

// Test: Boundary values - exceeds MAX_STEER_ANG
TEST_F(TestCalculateSteerRequest, ExceedsMaxSteerAngle) {
  // Input: greater than MAX_STEER_ANG -> should clamp to 100
  int8_t result = CalculateSteerRequest(1.0);
  EXPECT_EQ(result, 100);

  result = CalculateSteerRequest(10.0);
  EXPECT_EQ(result, 100);

  result = CalculateSteerRequest(100.0);
  EXPECT_EQ(result, 100);
}

// Test: Boundary values - below MIN_STEER_ANG
TEST_F(TestCalculateSteerRequest, BelowMinSteerAngle) {
  // Input: less than MIN_STEER_ANG -> should clamp to -100
  int8_t result = CalculateSteerRequest(-1.0);
  EXPECT_EQ(result, -100);

  result = CalculateSteerRequest(-10.0);
  EXPECT_EQ(result, -100);

  result = CalculateSteerRequest(-100.0);
  EXPECT_EQ(result, -100);
}

// Test: Return value range validation (-100 to 100)
TEST_F(TestCalculateSteerRequest, ReturnValueRangeValidation) {
  // Test various inputs and verify return is always within [-100, 100]
  std::vector<double> test_inputs = {
    -2.0, -1.5, -1.0, -0.8, -0.69813, -0.5, -0.3, -0.1, 0.0,
    0.1, 0.3, 0.5, 0.69813, 0.8, 1.0, 1.5, 2.0
  };

  for (double input : test_inputs) {
    int8_t result = CalculateSteerRequest(input);
    EXPECT_GE(result, -100);
    EXPECT_LE(result, 100)
      << "Result " << static_cast<int>(result) << " for input " << input
      << " is outside valid range [-100, 100]";
  }
}

// ============================================================================
// Test CalculateTorqueRequestTwizy - Has static state
// ============================================================================

class TestCalculateTorqueRequestTwizy : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test: Twizy - Initial state (first call with zero velocity)
TEST_F(TestCalculateTorqueRequestTwizy, InitialState) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Target 5 m/s, current 0 m/s - first call, stationary to accelerating
  int8_t result = CalculateTorqueRequestTwizy(
    5.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return a positive torque (accelerating)
  EXPECT_GE(result, 0);
  EXPECT_LE(result, MAX_TORQUE_TWIZY);

  // P contribution should be significant (error = 5.0, Kp = 23)
  EXPECT_GT(p_contrib, 0);
}

// Test: Twizy - Standstill should use brake hold torque
TEST_F(TestCalculateTorqueRequestTwizy, StandstillUsesBrakeHoldTorque) {
  int p_contrib = 123, i_contrib = 456, d_contrib = 789, ff_contrib = -1;

  int8_t result = CalculateTorqueRequestTwizy(
    0.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(result, BRAKE_HOLD_TORQUE_TWIZY);
  EXPECT_EQ(p_contrib, 0);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, BRAKE_HOLD_TORQUE_TWIZY);
}

// Test: Twizy - Negative targets should clamp to standstill
TEST_F(TestCalculateTorqueRequestTwizy, NegativeTargetWhileStationaryClampsToStandstill) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestTwizy(
    -2.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(result, BRAKE_HOLD_TORQUE_TWIZY);
  EXPECT_EQ(p_contrib, 0);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, BRAKE_HOLD_TORQUE_TWIZY);
}

// Test: Twizy - Stationary to acceleration transition
TEST_F(TestCalculateTorqueRequestTwizy, StationaryToAcceleration) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // First call: target 3 m/s, current 0 m/s
  int8_t result1 = CalculateTorqueRequestTwizy(
    3.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);
  EXPECT_GE(result1, 0);
  EXPECT_LE(result1, MAX_TORQUE_TWIZY);

  // Second call: target 3 m/s, current 1 m/s (still accelerating)
  int8_t result2 = CalculateTorqueRequestTwizy(
    3.0, 1.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);
  EXPECT_GE(result2, 0);
  EXPECT_LE(result2, MAX_TORQUE_TWIZY);
}

// Test: Twizy - Known acceleration path should produce stable controller terms
TEST_F(TestCalculateTorqueRequestTwizy, AccelerationCombinesProportionalAndFeedforwardTerms) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestTwizy(
    3.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, 69);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, 27);
  EXPECT_EQ(result, 96);
}

// Test: Twizy - Large requests should saturate at max torque
TEST_F(TestCalculateTorqueRequestTwizy, HighAccelerationDemandSaturatesAtMaxTorque) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestTwizy(
    20.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_GT(p_contrib, 0);
  EXPECT_GT(ff_contrib, 0);
  EXPECT_EQ(result, MAX_TORQUE_TWIZY);
}

// Test: Twizy - Deceleration to stop
TEST_F(TestCalculateTorqueRequestTwizy, DecelerationToStop) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Vehicle moving at 5 m/s, target is 0 (braking to stop)
  int8_t result = CalculateTorqueRequestTwizy(
    0.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return negative torque (braking)
  EXPECT_LE(result, 0);
  EXPECT_GE(result, MIN_TORQUE_TWIZY);
}

// Test: Twizy - Full stop request while moving should apply braking gains
TEST_F(TestCalculateTorqueRequestTwizy, StopRequestWhileMovingAppliesBrakingTorque) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestTwizy(
    0.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, -325);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, 0);
  EXPECT_EQ(result, MIN_TORQUE_TWIZY);
}

// Test: Twizy - Overspeed condition
TEST_F(TestCalculateTorqueRequestTwizy, OverspeedCondition) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Target 3 m/s, current 5 m/s (going too fast)
  int8_t result = CalculateTorqueRequestTwizy(
    3.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return negative torque (reducing speed)
  EXPECT_LE(result, 0);
  EXPECT_GE(result, MIN_TORQUE_TWIZY);
}

// Test: Twizy - Overspeed should use retardation gains
TEST_F(TestCalculateTorqueRequestTwizy, OverspeedUsesRetardationGains) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestTwizy(
    3.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, -80);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_LT(result, 0);
}

// Test: Twizy - Return value range validation
TEST_F(TestCalculateTorqueRequestTwizy, ReturnValueRangeValidation) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Various scenarios
  std::vector<std::pair<double, double>> test_cases = {
    {0.0, 0.0},   // stationary
    {5.0, 0.0},   // accelerating from stop
    {5.0, 5.0},   // constant speed
    {0.0, 5.0},   // braking to stop
    {3.0, 5.0},   // overspeed
    {10.0, 0.0},  // high speed request
  };

  for (const auto & tc : test_cases) {
    int8_t result = CalculateTorqueRequestTwizy(
      tc.first, tc.second, p_contrib, i_contrib,
      d_contrib, ff_contrib);
    EXPECT_GE(result, MIN_TORQUE_TWIZY)
      << "Result " << static_cast<int>(result) << " for target=" << tc.first
      << ", current=" << tc.second << " is below MIN_TORQUE_TWIZY";
    EXPECT_LE(result, MAX_TORQUE_TWIZY)
      << "Result " << static_cast<int>(result) << " for target=" << tc.first
      << ", current=" << tc.second << " is above MAX_TORQUE_TWIZY";
  }
}

// ============================================================================
// Test CalculateTorqueRequestEnv200 - Has static state
// ============================================================================

class TestCalculateTorqueRequestEnv200 : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// Test: Env200 - Initial state (first call with zero velocity)
TEST_F(TestCalculateTorqueRequestEnv200, InitialState) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Target 5 m/s, current 0 m/s - first call, stationary to accelerating
  int8_t result = CalculateTorqueRequestEnv200(
    5.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return a positive torque (accelerating)
  EXPECT_GE(result, 0);
  EXPECT_LE(result, MAX_TORQUE_ENV200);
}

// Test: Env200 - Standstill should use brake hold torque
TEST_F(TestCalculateTorqueRequestEnv200, StandstillUsesBrakeHoldTorque) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    0.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(result, BRAKE_HOLD_TORQUE_ENV200);
  EXPECT_EQ(p_contrib, 0);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, BRAKE_HOLD_TORQUE_ENV200);
}

// Test: Env200 - Negative targets should clamp to standstill
TEST_F(TestCalculateTorqueRequestEnv200, NegativeTargetWhileStationaryClampsToStandstill) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    -1.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(result, BRAKE_HOLD_TORQUE_ENV200);
  EXPECT_EQ(p_contrib, 0);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, BRAKE_HOLD_TORQUE_ENV200);
}

// Test: Env200 - Stationary to acceleration transition
TEST_F(TestCalculateTorqueRequestEnv200, StationaryToAcceleration) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // First call: target 3 m/s, current 0 m/s
  int8_t result1 = CalculateTorqueRequestEnv200(
    3.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);
  EXPECT_GE(result1, 0);
  EXPECT_LE(result1, MAX_TORQUE_ENV200);

  // Second call: target 3 m/s, current 1 m/s (still accelerating)
  int8_t result2 = CalculateTorqueRequestEnv200(
    3.0, 1.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);
  EXPECT_GE(result2, 0);
  EXPECT_LE(result2, MAX_TORQUE_ENV200);
}

// Test: Env200 - Known acceleration path should produce stable controller terms
TEST_F(TestCalculateTorqueRequestEnv200, AccelerationCombinesProportionalAndFeedforwardTerms) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    3.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, 21);
  EXPECT_EQ(i_contrib, 0);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, 6);
  EXPECT_EQ(result, 27);
}

// Test: Env200 - Large requests should saturate at max torque
TEST_F(TestCalculateTorqueRequestEnv200, HighAccelerationDemandSaturatesAtMaxTorque) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    20.0, 0.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_GT(p_contrib, 0);
  EXPECT_GT(ff_contrib, 0);
  EXPECT_EQ(result, MAX_TORQUE_ENV200);
}

// Test: Env200 - Deceleration to stop
TEST_F(TestCalculateTorqueRequestEnv200, DecelerationToStop) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Vehicle moving at 5 m/s, target is 0 (braking to stop)
  int8_t result = CalculateTorqueRequestEnv200(
    0.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return negative torque (braking)
  EXPECT_LE(result, 0);
  EXPECT_GE(result, MIN_TORQUE_ENV200);
}

// Test: Env200 - Full stop request while moving should apply braking gains
TEST_F(TestCalculateTorqueRequestEnv200, StopRequestWhileMovingAppliesBrakingTorque) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    0.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, -100);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_EQ(ff_contrib, 0);
  EXPECT_EQ(result, MIN_TORQUE_ENV200);
}

// Test: Env200 - Overspeed condition
TEST_F(TestCalculateTorqueRequestEnv200, OverspeedCondition) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Target 3 m/s, current 5 m/s (going too fast)
  int8_t result = CalculateTorqueRequestEnv200(
    3.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  // Should return negative torque (reducing speed)
  EXPECT_LE(result, 0);
  EXPECT_GE(result, MIN_TORQUE_ENV200);
}

// Test: Env200 - Overspeed should use retardation gains
TEST_F(TestCalculateTorqueRequestEnv200, OverspeedUsesRetardationGains) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  int8_t result = CalculateTorqueRequestEnv200(
    3.0, 5.0, p_contrib, i_contrib, d_contrib,
    ff_contrib);

  EXPECT_EQ(p_contrib, -30);
  EXPECT_EQ(d_contrib, 0);
  EXPECT_LT(result, 0);
}

// Test: Env200 - Return value range validation
TEST_F(TestCalculateTorqueRequestEnv200, ReturnValueRangeValidation) {
  int p_contrib = 0, i_contrib = 0, d_contrib = 0, ff_contrib = 0;

  // Various scenarios
  std::vector<std::pair<double, double>> test_cases = {
    {0.0, 0.0},   // stationary
    {5.0, 0.0},   // accelerating from stop
    {5.0, 5.0},   // constant speed
    {0.0, 5.0},   // braking to stop
    {3.0, 5.0},   // overspeed
    {10.0, 0.0},  // high speed request
  };

  for (const auto & tc : test_cases) {
    int8_t result = CalculateTorqueRequestEnv200(
      tc.first, tc.second, p_contrib, i_contrib,
      d_contrib, ff_contrib);
    EXPECT_GE(result, MIN_TORQUE_ENV200)
      << "Result " << static_cast<int>(result) << " for target=" << tc.first
      << ", current=" << tc.second << " is below MIN_TORQUE_ENV200";
    EXPECT_LE(result, MAX_TORQUE_ENV200)
      << "Result " << static_cast<int>(result) << " for target=" << tc.first
      << ", current=" << tc.second << " is above MAX_TORQUE_ENV200";
  }
}

}  // namespace speedcontroller

// Main function for running tests
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
