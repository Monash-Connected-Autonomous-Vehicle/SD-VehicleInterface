// Copyright (c) 2020 StreetDrone Limited
//
//
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
//    * Neither the name of the StreetDrone Limited nor the names of its
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

// Author: Fionán O'Sullivan
// Based on original work of: Efimia Panagiotaki

#ifndef SD_VEHICLE_INTERFACE__SD_VEHICLE_INTERFACE_H_
#define SD_VEHICLE_INTERFACE__SD_VEHICLE_INTERFACE_H_

#include <cstdint>
#include <string>
#include <can_msgs/msg/frame.hpp>
// *****CONSTANTS*****

// Constants
#define  RAD_to_DEG  (57.2958)  // Conversion constant from rad to deg.
#define  DEG_to_RAD  (0.0174533)  // Conversion constant from deg to rad.
const double UNDO_STREETDRONE_SCALING_FACTOR = 50;
// MCAV note: this converts reported speed to meters per second.

// Ros frequencies
#define ROS_LOOP (200)  // The rate at which we publish data.

// Control frequencies
#define CONTROL_LOOP (20)  // 200 / 20 = 5 Hz.

// *****VARIABLES*****
double CurrentTwistLinearCANImu_Mps = 0.0;  // Current twist linear from GPS/IMU CAN bus.
double CurrentTwistLinearSD_Mps_Final = 0.0;  // Final speed source used by control loops.
double CurrentTwistLinearCANSD_Mps = 0.0;  // Current twist linear from StreetDrone XCU CAN.
double CurrentTwistLinearNDT_Mps = 0.0;  // Current twist linear from NDT localization.
double GPS_Latitude = 0.0;  // Latitude from CAN bus.
double GPS_Longitude = 0.0;  // Longitude from CAN bus.
double IMU_Angle_X = 0;
double IMU_Angle_Y = 0;
double IMU_Angle_Z = 0;
double IMU_Rate_X = 0;
double IMU_Rate_Y = 0;
double IMU_Rate_Z = 0;
double IMU_Accel_X = 0;
double IMU_Accel_Y = 0;
double IMU_Accel_Z = 0;
double TargetTwistLinear_Mps;  // Target twist linear in m/s.
double TargeTireAngle_Rad;  // Target steering tire angle in radians.
uint8_t AliveCounter_Z = 0;  // Alive counter increments every cycle.
bool AutomationArmed_B = false;  // True when safety driver arms autonomous mode.
bool AutomationGranted_B = false;  // True when vehicle grants autonomous request.
bool IMUVarianceKnown_B = false;  // True if IMU covariance is known (OXTS yes, PEAK no).


// Speed control
int8_t FinalDBWTorqueRequest_Pc = 0;  // Final torque request, -100 (brake) to 100.
int8_t FinalDBWSteerRequest_Pc = 0;  // Final steer request, +/-100 full lock.
int P_Contribution_Pc = 0;  // Torque contributed by proportional gain.
int I_Contribution_Pc = 0;  // Torque contributed by integral gain.
int D_Contribution_Pc = 0;  // Torque contributed by derivative gain.
int FF_Contribution_Pc = 0;  // Torque contributed by feedforward gain.

// ROS variables
can_msgs::msg::Frame ReceivedFrameCANRx;  // Latest received CAN frame.
can_msgs::msg::Frame CustomerControlCANTx;  // CAN frame sent to vehicle control bus.
can_msgs::msg::Frame ControllerFeedbackCANTx;  // Optional controller feedback frame.

// ros::Publisher sent_msgs_pub;
// ros::Publisher current_twist_pub;
// ros::Publisher current_GPS_pub;
// ros::Publisher current_IMU_pub;
// ros::Publisher sd_control_pub;


std::string _sd_vehicle;  // NOLINT(runtime/string)
std::string _sd_gps_imu;  // NOLINT(runtime/string)
std::string _sd_speed_source;  // NOLINT(runtime/string)
static bool _sd_simulation_mode;

const char * const twizy_string = "twizy";

const char * const oxts_string = "oxts";
const char * const peak_string = "peak";
const char * const no_imu_string = "none";

const char * const vehicle_can_speed_string = "vehicle_can_speed";
const char * const imu_speed_string = "imu_speed";
const char * const ndt_speed_string = "ndt_speed";

#endif  // SD_VEHICLE_INTERFACE__SD_VEHICLE_INTERFACE_H_
