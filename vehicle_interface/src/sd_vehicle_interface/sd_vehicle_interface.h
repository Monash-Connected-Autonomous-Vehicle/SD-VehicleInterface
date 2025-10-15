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

#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "tier4_control_msgs/msg/gate_mode.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include <cstdint>
#include <string>

// ===== CONSTANTS =====

// Constants
#define RAD_to_DEG (57.2958)   // Conversion constant from rad to deg
#define DEG_to_RAD (0.0174533) // Conversion constant from deg to rad
// MCAV note: we create this to convert the reported speed to
// actually be in meters per second. Previously, the code was not
// correctly interpreting the CAN frames
const double kUndoSDScalingFactor = 50;

// rate at which we publish data
#define ROS_LOOP (200)

// number of cycles counted for each control loop cycle (200/20 = 10Hz)
#define CONTROL_LOOP (20)

// ===== FUNCTION SIGNATURES =====

void ControlCmdCallback(
    const shared_ptr<autoware_control_msgs::msg::Control> msg);

void HazardCmdCallback(
    const shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> msg);

void IndicatorsCmdCallback(
    const shared_ptr<autoware_vehicle_msgs::msg::TurnIndicatorsCommand> msg);

void GearCmdCallback(
    const shared_ptr<autoware_vehicle_msgs::msg::GearCommand> msg);

void GateModeCmdCallback(
    const shared_ptr<tier4_control_msgs::msg::GateMode> msg);

void EmergencyCmdCallback(
    const shared_ptr<tier4_vehicle_msgs::msg::VehicleEmergencyStamped> msg);

void ActuationCmdCallback(
    const shared_ptr<tier4_vehicle_msgs::msg::ActuationCommandStamped> msg);

// ===== VARIABLES =====

// current twist (m/s), read from GPS/IMU
double current_twist_linear_can_imu_mps = 0.0;
// speed used within control functions.
// specified as based on CAN/IMU/localisation at launch
double current_twist_linear_sd_mps_final = 0.0;
double current_twist_linear_can_sd_mps =
    0.0; // Current Twist Linear in Mps, as read from the CAN bus from the
         // StreetDrone XCU
int8_t current_steer_pc = 0;
double current_twist_linear_ndt_mps =
    0.0; // Current Twist Linear in Mps, as reported by NDT locolisation
double gps_latitude = 0.0;  // latitude, as read from the CAN bus
double gps_longitude = 0.0; // latitude, as read from the CAN bus
double imu_angle_x = 0;
double imu_angle_y = 0;
double imu_angle_z = 0;
double imu_rate_x = 0;
double imu_rate_y = 0;
double imu_rate_z = 0;
double imu_accel_x = 0;
double imu_accel_y = 0;
double imu_accel_z = 0;

// === Ackermann Targets ===

double target_twist_linear_mps;          // Target Twist linear in m/s
double target_tire_angle_rad;            // Target Twist angular in deg/s
double target_steering_tire_rotation_rate; // Steering angle rate of change (rad/s)

// Auxiliary control
uint8_t target_hazard_lights_cmd; // Hazard lights command received from autoware
uint8_t target_indicators_cmd;   // Indicators command received from autoware
uint8_t target_gear_cmd;         // Gear command received from autoware

// Other Autoware Control
uint8_t target_gate_mode_cmd;
bool is_emergency;
double target_accel_cmd_temp;
double target_brake_cmd_temp;
double target_steer_cmd_temp;

// Requests populated to CAN frame to vehicle
bool final_hazard_lights_request;
bool final_indicator_left_request;
bool final_indicator_right_request;

//
uint8_t alive_counter_z = 0;     // Alive Counter, increments every cycle
bool automation_armed_b = false; // Boolean, true if safety driver turns mode
                                // switch to autonomous mode
bool automation_granted_b =
    false; // Boolean, true if vehicle grants autonomous mode request
bool imu_variance_known_b = false; // Boolean, true if the inout GPS has known
                                 // variance (OXTS YES, PEAK NO)

// Speed Control
int8_t final_dbw_torque_request_pc =
    0; // The Final Drive-By-Wire torque request, expressed from -100% (full
       // brake) to 100% (full throttle)
int8_t final_dbw_steer_request_pc =
    0; // Final steer request, +/- 100 is full lock left and right
int p_contribution_pc = 0;  // The torque contributed by proportional gain
int i_contribution_pc = 0;  // The torque contributed  by integral gain
int d_contribution_pc = 0;  // The torque contributed by derivative gain
int ff_contribution_pc = 0; // The torque contributed by feedforward gain

// Ros variables

// CAN frame received from vehicle
can_msgs::msg::Frame received_can_rx_frame;
// Customer_Control_1 CAN frame (0x101), sent to CAN bus
can_msgs::msg::Frame customer_control_can_tx;
// Customer_Control_2 CAN frame (0x104), sent to CAN bus
can_msgs::msg::Frame customer_control_auxiliary_can_tx;
// feedback CAN frame (0x103), supplies data to tune controller (UNUSED)
can_msgs::msg::Frame ControllerFeedbackCANTx;

// ros::Publisher sent_msgs_pub;
// ros::Publisher current_twist_pub;
// ros::Publisher current_gps_pub;
// ros::Publisher current_imu_pub;
// ros::Publisher sd_control_pub;

static string _sd_vehicle;
static string _sd_gps_imu;
static string _sd_speed_source;
static bool _sd_simulation_mode;

static string twizy_string = "twizy";

static string oxts_string = "oxts";
static string peak_string = "peak";
static string no_imu_string = "none";

static string vehicle_can_speed_string = "vehicle_can_speed";
static string imu_speed_string = "imu_speed";
static string ndt_speed_string = "ndt_speed";
