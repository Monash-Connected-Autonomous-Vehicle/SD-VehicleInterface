/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 *
 * Author: Fionán O'Sullivan
 *
 * Based on original work of: Efimia Panagiotaki
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

using namespace std;

#include "sd_control.h"

namespace speedcontroller {

// linear velocity error from the last cycle
static double PreviousLinearVelocityError_Mps = 0;

// column units m/s, rows deg/s
// Map should be extended if tighter turns needed above 6m/s

uint8_t steer_map[YAW_YAXIS][V_XAXIS] = {
    //0  <1   <2   <3   <4   <5   <6   <7  <8  <9
    {0,   0,   0,   0,   0,   0,   0,   0,  0,  0},  // 0
    {40,  30,  20,  18,  11,  10,  7,   7,  6,  3},  // <5
    {60,  50,  37,  30,  21,  13,  14,  12, 11, 7},  // <10
    {95,  65,  49,  43,  34,  28,  21,  16, 14, 12}, // <15
    {100, 97,  90,  77,  50,  35,  28,  23, 21, 17}, // <20
    {100, 100, 100, 90,  55,  39,  35,  28, 25, 17}, // <25
    {100, 100, 100, 100, 60,  50,  40,  30, 28, 17}, // <30
    {100, 100, 100, 100, 65,  60,  45,  35, 32, 17}, // <35
    {100, 100, 100, 100, 90,  70,  58,  42, 38, 17}, // <40
    {100, 100, 100, 100, 95,  85,  65,  48, 33, 17}, // <45
    {100, 100, 100, 100, 100, 90,  70,  55, 45, 17}, // <50
    {100, 100, 100, 100, 100, 95,  80,  60, 50, 19}, // <55
    {100, 100, 100, 100, 100, 100, 90,  65, 53, 20}, // <60
    {100, 100, 100, 100, 100, 100, 100, 70, 55, 45}, // >60
};

// FeedForward Calibration
// Provide FF torque (% from 0-100) for a given speed. An additional axis
// should be considered for gradient. Note: FF gain at 0 = BRAKE_HOLD_TORQUE
int8_t feedforward_torque_map_twizy[V_XAXIS] =
//>0m/s <1m/s <2m/s <3m/s <4m/s <5m/s <6m/s <7m/s <8m/s <9m/s
  {33,   33,   33,   34,   35,   38,   39,   40,   41,   42};

int8_t feedforward_torque_map_env200[V_XAXIS] =
//>0m/s <1m/s <2m/s <3m/s <4m/s <5m/s <6m/s <7m/s <8m/s <9m/s
  {-5,   0,    2,    6,    7,   8,    10,   12,   13,   15};

int8_t CalculateSteerRequest(double TargetSteerAngle_Rad) {
  float angle_request_rad = TargetSteerAngle_Rad;

  // Clamp angle request
  if (angle_request_rad > MAX_STEER_ANG) {
    angle_request_rad = MAX_STEER_ANG;
  } else if (angle_request_rad < MIN_STEER_ANG) {
    angle_request_rad = MIN_STEER_ANG;
  }

  int CalculatedSteeringAngle_Pc = (angle_request_rad / MAX_STEER_ANG) * 100;
  return CalculatedSteeringAngle_Pc;
}

/*
 * Calculate torque request for Twizy given actual and target speeds.
 * Accounts for PID controller output and FeedForward term.
 */
int8_t CalculateTorqueRequestTwizy(double TargetLinearVelocity_Mps,
                                   double CurrentLinearVelocity_Mps,
                                   int &p_contribution_pc,
                                   int &i_contribution_pc,
                                   int &d_contribution_pc,
                                   int &ff_contribution_pc) {

  // stores PID Errors - static as needed to persist across multiple calls
  static double LinearVelocityError_Mps;
  static double LinearVelocityIntegratedError;
  static double LinearVelocityDerivativeError;

  // final torque request to send to DBW system
  int final_dbw_torque_request_pc;

  // Speed index and remainder used to interpolate between different speed
  // values of the steer table
  int speed_index = floor(TargetLinearVelocity_Mps);
  double speed_index_remainder = abs(TargetLinearVelocity_Mps) - speed_index;

  if (speed_index > V_XAXIS - 2) {
    speed_index = V_XAXIS - 2;
  }

  // Reverse currently not suppported. If a negative speed request is
  // received, we clamp it to 0 (standstill)
  if (TargetLinearVelocity_Mps <= 0) {
    TargetLinearVelocity_Mps = 0;
    ff_contribution_pc = 0;
  } else {
    // Calculate Feedforward contribution
    // ff_contribution_pc  =
    // (1-speed_index_remainder)*feedforward_torque_map_twizy[speed_index] +
    // speed_index_remainder*feedforward_torque_map_twizy[speed_index+1];
    ff_contribution_pc =
        ((1 - speed_index_remainder) *
             feedforward_torque_map_twizy[speed_index] +
         speed_index_remainder * feedforward_torque_map_twizy[speed_index + 1]);
  }

  // Calculate error for PID
  LinearVelocityError_Mps =
      TargetLinearVelocity_Mps - CurrentLinearVelocity_Mps;
  LinearVelocityDerivativeError =
      LinearVelocityError_Mps - PreviousLinearVelocityError_Mps;
  LinearVelocityIntegratedError =
      LinearVelocityIntegratedError + LinearVelocityError_Mps;

  if ((TargetLinearVelocity_Mps == 0 &&
       CurrentLinearVelocity_Mps == 0)) { // not moving
    p_contribution_pc = 0;
    i_contribution_pc = 0;
    d_contribution_pc = 0;
    ff_contribution_pc = BRAKE_HOLD_TORQUE_TWIZY;
  } else if (abs(LinearVelocityError_Mps) <
             ANTI_FUSSINESS_TWIZY) { // within error range

    // PID remain the same. I gain = 0. This resets the I gain so we don't
    //  have to "unwind" LinearVelocityIntegratedError = 0; i_contribution_pc
    //  = 0;
    // P, D and FF fails maintain last value

    LinearVelocityIntegratedError = 0.975 * LinearVelocityIntegratedError;
    i_contribution_pc = 0.975 * i_contribution_pc;

  } else if ((TargetLinearVelocity_Mps == 0 &&
              CurrentLinearVelocity_Mps > 0)) { // trying to stop
    p_contribution_pc =
        LinearVelocityError_Mps * Kp_Speed_FullStop_Braking_Twizy;
    i_contribution_pc =
        LinearVelocityIntegratedError * Ki_Speed_FullStop_Braking_Twizy;
    d_contribution_pc =
        LinearVelocityDerivativeError * Kd_Speed_FullStop_Braking_Twizy;

  } else if (LinearVelocityError_Mps < -ANTI_FUSSINESS_TWIZY) { 
    // going too fast (account for vehicle overrun/coasting)
    p_contribution_pc = LinearVelocityError_Mps * Kp_Speed_Retd_Twizy;
    i_contribution_pc = LinearVelocityIntegratedError * Ki_Speed_Retd_Twizy;
    d_contribution_pc = LinearVelocityDerivativeError * Kd_Speed_Retd_Twizy;

  } else { // too slow
    p_contribution_pc = LinearVelocityError_Mps * Kp_Speed_Twizy;
    i_contribution_pc = LinearVelocityIntegratedError * Ki_Speed_Twizy;
    d_contribution_pc = LinearVelocityDerivativeError * Kd_Speed_Twizy;
  }

  // I gain Anti windup Strategy
  if (i_contribution_pc > MAX_ABS_I_CONTRIBUTION_TWIZY) { // I Gain saturation
    i_contribution_pc = MAX_ABS_I_CONTRIBUTION_TWIZY;
  } else if (i_contribution_pc < -MAX_ABS_I_CONTRIBUTION_TWIZY) {
    i_contribution_pc = -MAX_ABS_I_CONTRIBUTION_TWIZY;
  }

  // Prevents I gain winding up when sitting still with handbrake on
  if (CurrentLinearVelocity_Mps < ANTI_FUSSINESS_TWIZY) {
    i_contribution_pc = 0;
  }

  // I Gain should only influence the system in a band about the setpoint.
  if (abs(LinearVelocityError_Mps) > I_GAIN_ERROR_BAND_TWIZY) {
    LinearVelocityIntegratedError = 0;
    i_contribution_pc = 0;
  }

  final_dbw_torque_request_pc = p_contribution_pc + i_contribution_pc +
                             d_contribution_pc + ff_contribution_pc;

  // Clamp torque
  if (final_dbw_torque_request_pc > MAX_TORQUE_TWIZY) {
    final_dbw_torque_request_pc = MAX_TORQUE_TWIZY;
  } else if (final_dbw_torque_request_pc < MIN_TORQUE_TWIZY) {
    final_dbw_torque_request_pc = MIN_TORQUE_TWIZY;
  }

  // Remember previous variables for next cycle
  PreviousLinearVelocityError_Mps = LinearVelocityError_Mps;

  return (int8_t)final_dbw_torque_request_pc;
}

/*
 * Calculate torque request for ENV200 given actual and target speeds.
 * Accounts for PID controller output and FeedForward term.
 */
int8_t CalculateTorqueRequestEnv200(double TargetLinearVelocity_Mps,
                                    double CurrentLinearVelocity_Mps,
                                    int &p_contribution_pc,
                                    int &i_contribution_pc,
                                    int &d_contribution_pc,
                                    int &ff_contribution_pc) {

  // stores PID errors - static as needed to persist across multiple calls
  static double LinearVelocityError_Mps;
  static double LinearVelocityIntegratedError;
  static double LinearVelocityDerivativeError;

  // final torque request to send to DBW system
  int final_dbw_torque_request_pc;

  // Speed index and remainder used to interpolate between different speed
  // values of the steer table
  int speed_index = floor(TargetLinearVelocity_Mps);
  double speed_index_remainder = abs(TargetLinearVelocity_Mps) - speed_index;

  if (speed_index > V_XAXIS - 2) {
    speed_index = V_XAXIS - 2;
  }

  // Reverse currently not suppported. If a negative speed request is
  // received, we clamp it to 0 (standstill)
  if (TargetLinearVelocity_Mps <= 0) {
    TargetLinearVelocity_Mps = 0;
    ff_contribution_pc = 0;
  } else {
    // Calculate Feedforward contribution
    ff_contribution_pc =
        (1 - speed_index_remainder) *
            feedforward_torque_map_env200[speed_index] +
        speed_index_remainder * feedforward_torque_map_env200[speed_index + 1];
  }

  // Calculate error for PID
  LinearVelocityError_Mps =
      TargetLinearVelocity_Mps - CurrentLinearVelocity_Mps;
  LinearVelocityDerivativeError =
      LinearVelocityError_Mps - PreviousLinearVelocityError_Mps;
  LinearVelocityIntegratedError =
      LinearVelocityIntegratedError + LinearVelocityError_Mps;

  if ((TargetLinearVelocity_Mps == 0 &&
       CurrentLinearVelocity_Mps == 0)) { // not moving
    p_contribution_pc = 0;
    i_contribution_pc = 0;
    d_contribution_pc = 0;
    ff_contribution_pc = BRAKE_HOLD_TORQUE_ENV200;

  } else if (abs(LinearVelocityError_Mps) <
             ANTI_FUSSINESS_ENV200) { // within error range
    // PID remain the same. I gain = 0. This resets the I gain so we don't
    // have to "unwind"
    LinearVelocityIntegratedError = 0;
    i_contribution_pc = 0;
    // P, D and FF fails maintain last value

  } else if ((TargetLinearVelocity_Mps == 0 &&
              CurrentLinearVelocity_Mps > 0)) { // trying to stop
    p_contribution_pc =
        LinearVelocityError_Mps * Kp_Speed_FullStop_Braking_Env200;
    i_contribution_pc =
        LinearVelocityIntegratedError * Ki_Speed_FullStop_Braking_Env200;
    d_contribution_pc =
        LinearVelocityDerivativeError * Kd_Speed_FullStop_Braking_Env200;

  } else if (LinearVelocityError_Mps <
             -ANTI_FUSSINESS_ENV200) { // going to fast (account for vehicle
                                       // overrun/coasting)
    p_contribution_pc = LinearVelocityError_Mps * Kp_Speed_Retd_Env200;
    i_contribution_pc = LinearVelocityIntegratedError * Ki_Speed_Retd_Env200;
    d_contribution_pc = LinearVelocityDerivativeError * Kd_Speed_Retd_Env200;

  } else { // too slow
    p_contribution_pc = LinearVelocityError_Mps * Kp_Speed_Env200;
    i_contribution_pc = LinearVelocityIntegratedError * Ki_Speed_Env200;
    d_contribution_pc = LinearVelocityDerivativeError * Kd_Speed_Env200;
  }

  // I gain Anti windup Strategy
  if (i_contribution_pc > MAX_ABS_I_CONTRIBUTION_ENV200) { // I Gain saturation
    i_contribution_pc = MAX_ABS_I_CONTRIBUTION_ENV200;
  } else if (i_contribution_pc < -MAX_ABS_I_CONTRIBUTION_ENV200) {
    i_contribution_pc = -MAX_ABS_I_CONTRIBUTION_ENV200;
  }

  // Prevents I gain winding up when sitting still with handbrake on
  if (CurrentLinearVelocity_Mps < ANTI_FUSSINESS_ENV200) {
    i_contribution_pc = 0;
  }

  // I Gain should only influence the system in a band about the setpoint.
  if (abs(LinearVelocityError_Mps) > I_GAIN_ERROR_BAND_ENV200) {
    LinearVelocityIntegratedError = 0;
    i_contribution_pc = 0;
  }

  final_dbw_torque_request_pc = p_contribution_pc + i_contribution_pc +
                             d_contribution_pc + ff_contribution_pc;

  // Clamp torque
  if (final_dbw_torque_request_pc > MAX_TORQUE_ENV200) {
    final_dbw_torque_request_pc = MAX_TORQUE_ENV200;
  } else if (final_dbw_torque_request_pc < MIN_TORQUE_ENV200) {
    final_dbw_torque_request_pc = MIN_TORQUE_ENV200;
  }

  // Remember previous variables for next cycle
  PreviousLinearVelocityError_Mps = LinearVelocityError_Mps;

  return (int8_t)final_dbw_torque_request_pc;
}

} // namespace speedcontroller
