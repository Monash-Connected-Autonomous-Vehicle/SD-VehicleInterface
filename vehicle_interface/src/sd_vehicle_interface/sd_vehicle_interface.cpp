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

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "can_msgs/msg/frame.hpp"

#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"

#include "sd_auxiliary_controller.h"
#include "sd_control.h"
#include "sd_gps_imu.h"
#include "sd_lib_mcav.h"
#include "sd_msgs/msg/sd_control.hpp"
#include "sd_vehicle_interface.h"

using autoware_vehicle_msgs::msg::ControlModeReport;

// Stating variables for automation modes
uint8_t steer_automation_state  = 0;
uint8_t torque_automation_state = 0;

// Constants
namespace {
  // Default values
  constexpr int kDefaultEnergyLevel        = 100;

  // Math and timing
  constexpr double kLoopCalcDelaySec       = 0.1;
  constexpr int kMainLoopMs                = 5;
  constexpr double kYawDtSec               = 0.05;
  constexpr double kDegToRad               = M_PI / 180.0;

  // Queue depths
  constexpr int kToCanQueueDepth           = 100;
  constexpr int kFromCanQueueDepth         = 100; // Queue depth for incoming control e.g. CAN, commands
  constexpr int kTwistQueueDepth           = 100;
  constexpr int kGpsQueueDepth             = 100;
  constexpr int kImuQueueDepth             = 100;
  constexpr int kCurrentVelocityQueueDepth = 1;
  constexpr int kSdControlQueueDepth       = 1;
  constexpr int kVehicleStatusQueue        = 10;

  // Autonomous machine state codes
  constexpr int kAutoModeUnknown = 0;
  constexpr int kAutoModeInit = 1;
  constexpr int kAutoModeNoCan2 = 2;
  constexpr int kAutoModeManual = 3;
  constexpr int kAutoModeAutoSetup = 4;
  constexpr int kAutoModeAutoSetAvailable = 5; // Not currently referenced
  constexpr int kAutoModeAutoRequested = 6;
  constexpr int kAutoModeAutoGranted = 7;
  constexpr int kAutoModeAutoCleanup = 8; // Not currently referenced
  constexpr int kAutoModeMaxValue = 8;

} // end namespace

// ===== HELPER / CALLBACK FUNCTIONS =====

/**
 * Process incoming CAN message, parse data into variables
 *
 * Inputs:
 *   - msg: shared pointer to received CAN frame
 */
void OnCanRxFrame(const std::shared_ptr<const can_msgs::msg::Frame> msg) {

  // copy CAN frame into received_can_rx_frame
  received_can_rx_frame = *msg.get();

  // get current speed, automation status flags and autonomation states
  sd::ParseSdCanRxFrame(received_can_rx_frame, current_twist_linear_can_sd_mps, current_steer_pc,
                          automation_armed_b, automation_granted_b, steer_automation_state, torque_automation_state);

  // parse data depending on IMU/GPS device used
  if (oxts_string == _sd_gps_imu) {
    imu_variance_known_b = true; // variance/covariance known for OXTS
    // parse using OXTS function
    sd::ParseRxCanDataOxtsCan(
        received_can_rx_frame, current_twist_linear_can_imu_mps, gps_longitude,
        gps_latitude, imu_angle_x, imu_angle_y, imu_angle_z, imu_rate_x,
        imu_rate_y, imu_rate_z, imu_accel_x, imu_accel_y, imu_accel_z);
  } else if (peak_string == _sd_gps_imu) {
    // parse using PEAK function
    imu_variance_known_b = false; // variance/covariance not known for PEAK
    sd::ParseRxCanDataPeakCan(
        received_can_rx_frame, current_twist_linear_can_imu_mps, gps_longitude,
        gps_latitude, imu_angle_x, imu_angle_y, imu_angle_z, imu_rate_x,
        imu_rate_y, imu_rate_z, imu_accel_x, imu_accel_y, imu_accel_z);
  } else if (no_imu_string == _sd_gps_imu) {
    // no IMU, do nothing
  } else {
    // unknown IMU, warning
    // RCLCPP_WARN(node->get_logger(), "SD_Vehicle_Interface parameter for
    // sd_gps_imu is not valid\n");
  }
}

/**
 * Extract current forward speed from NDT (speed source)
 */
void CurrentVelocityCallback(
    const std::shared_ptr<const geometry_msgs::msg::TwistStamped> msg) {
  // Current Velocity Reported from NDT
  current_twist_linear_ndt_mps = msg->twist.linear.x; // mps to kph
}

// ===== AUTOWARE PUB/SUB =====
void ControlCmdCallback(
    const std::shared_ptr<autoware_control_msgs::msg::Control> msg) {
  target_tire_angle_rad = msg->lateral.steering_tire_angle;
  target_twist_linear_mps =
      msg->longitudinal.velocity / kUndoSDScalingFactor;
  target_steering_tire_rotation_rate = msg->lateral.steering_tire_rotation_rate;
}

void HazardCmdCallback(
    const std::shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> msg) {
  target_hazard_lights_cmd = msg->command;
}

void IndicatorsCmdCallback(
    const std::shared_ptr<autoware_vehicle_msgs::msg::TurnIndicatorsCommand> msg) {
  target_indicators_cmd = msg->command;
}

void GearCmdCallback(
    const std::shared_ptr<autoware_vehicle_msgs::msg::GearCommand> msg) {
  target_gear_cmd = msg->command;
}

void GateModeCmdCallback(
    const std::shared_ptr<tier4_control_msgs::msg::GateMode> msg) {
  target_gate_mode_cmd = msg->data;
}

void EmergencyCmdCallback(
    const std::shared_ptr<tier4_vehicle_msgs::msg::VehicleEmergencyStamped> msg) {
  is_emergency = msg->emergency;
}

void ActuationCmdCallback(
    const std::shared_ptr<tier4_vehicle_msgs::msg::ActuationCommandStamped> msg) {
  target_accel_cmd_temp= msg->actuation.accel_cmd;
  target_brake_cmd_temp= msg->actuation.brake_cmd;
  target_steer_cmd_temp = msg->actuation.steer_cmd;
}

double ComputeLateralVelocity(double vehicle_speed_mps,
                              double gps_course_deg,
                              double integrated_yaw_deg) {
  double course_rad = gps_course_deg * kDegToRad;
  double yaw_rad = integrated_yaw_deg * kDegToRad;

  return vehicle_speed_mps * std::sin(course_rad - yaw_rad);
}


// ===== MAIN FUNCTION =====

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  // ===== INITIALISE REQUIRED OBJECTS =====

  // create node + declare parameters:
  //   "sd_vehicle", "sd_gps_imu", "sd_speed_source", "sd_simulation_mode"
  auto node = rclcpp::Node::make_shared("sd_twizy_interface_node");
  node->declare_parameter<std::string>("sd_vehicle", "env200");
  _sd_vehicle = node->get_parameter("sd_vehicle").as_string();
  node->declare_parameter<std::string>("sd_gps_imu", "oxts");
  _sd_gps_imu = node->get_parameter("sd_gps_imu").as_string();
  node->declare_parameter<std::string>("sd_speed_source", "vehicle_can_speed");
  _sd_speed_source = node->get_parameter("sd_speed_source").as_string();
  node->declare_parameter<bool>("sd_simulation_mode", false);
  _sd_simulation_mode = node->get_parameter("sd_simulation_mode").as_bool();

  // initialise CAN variables
  sd::InitialiseSdInterfaceControl(customer_control_can_tx);     // Customer_Control_1
  sd::InitialiseSdInterfaceFeedback(ControllerFeedbackCANTx); // receive feedback data
  sd::InitialiseSdInterfaceControl_2(
      customer_control_auxiliary_can_tx); // Customer_Control_2

  // message objects (stores incoming data)
  geometry_msgs::msg::TwistStamped current_twist; // speed + steer
  sensor_msgs::msg::NavSatFix current_gps;    // latitude/longitude + altitude
  sensor_msgs::msg::Imu current_imu;          // acceleration + angular rate
  sd_msgs::msg::SDControl sd_current_control; // vehicle control message

  // Subscribers
  // store messages from vehicle
  auto received_can_rx_frame_sub = node->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", kFromCanQueueDepth, OnCanRxFrame);
  auto current_velocity_sub =
      node->create_subscription<geometry_msgs::msg::TwistStamped>(
          "current_velocity", kCurrentVelocityQueueDepth, CurrentVelocityCallback);
  
  // Autoware-specific subscribers
  auto control_sub =
      node->create_subscription<autoware_control_msgs::msg::Control>(
          "/control/command/control_cmd", kFromCanQueueDepth, ControlCmdCallback);
  auto gear_sub =
      node->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
          "/control/command/gear_cmd", kFromCanQueueDepth, GearCmdCallback);
  auto gate_mode_sub =
      node->create_subscription<tier4_control_msgs::msg::GateMode>(
          "/control/current_gate_mode", kFromCanQueueDepth, GateModeCmdCallback);
  auto emergency_cmd_sub =
      node->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
          "/control/command/emergency_cmd", kFromCanQueueDepth, EmergencyCmdCallback);
  auto actuation_cmd_sub =
      node->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
          "/control/command/actuation_cmd", kFromCanQueueDepth, ActuationCmdCallback);
  auto hazard_lights_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", kFromCanQueueDepth, HazardCmdCallback);
  auto indicators_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", kFromCanQueueDepth,
      IndicatorsCmdCallback);

  // Publishers
  // control commands (for ENV200)
  auto sent_msgs_pub =
      node->create_publisher<can_msgs::msg::Frame>("to_can_bus", kToCanQueueDepth);

  // current velocity
  auto current_twist_pub =
      node->create_publisher<geometry_msgs::msg::TwistStamped>(
          "sd_current_twist", kTwistQueueDepth);
  auto current_gps_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(
      "sd_current_gps", kGpsQueueDepth);
  auto current_imu_pub =
      node->create_publisher<sensor_msgs::msg::Imu>("sd_imu_raw", kImuQueueDepth);
  auto sd_control_pub =
      node->create_publisher<sd_msgs::msg::SDControl>("sd_control", kSdControlQueueDepth);
  
  // Autoware-specific publishers and message stores
  tier4_vehicle_msgs::msg::BatteryStatus temp_battery_status;
  temp_battery_status.energy_level = kDefaultEnergyLevel; 
  auto battery_status_pub =
      node->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("vehicle/status/battery_charge", kVehicleStatusQueue);

  autoware_vehicle_msgs::msg::ControlModeReport current_control_mode_report;
  current_control_mode_report.mode = ControlModeReport::AUTONOMOUS; 
  auto control_mode_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("vehicle/status/control_mode", kVehicleStatusQueue);
  
  autoware_vehicle_msgs::msg::GearReport current_gear_status;
  auto gear_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::GearReport>("vehicle/status/gear_status", kVehicleStatusQueue);
  
  autoware_vehicle_msgs::msg::HazardLightsReport current_hazard_lights_status;
  auto hazard_light_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("vehicle/status/hazard_lights_status", kVehicleStatusQueue);

  autoware_vehicle_msgs::msg::TurnIndicatorsReport current_indicator_status;
  auto indicator_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("vehicle/status/turn_indicators_status", kVehicleStatusQueue);

  autoware_vehicle_msgs::msg::SteeringReport current_steering_status;
  auto steering_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("vehicle/status/steering_status", kVehicleStatusQueue);

  autoware_vehicle_msgs::msg::VelocityReport current_velocity_status;
  auto velocity_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("vehicle/status/velocity_status", kVehicleStatusQueue);


  // set frequency of main loop (Hz)
  rclcpp::Rate loop_rate(ROS_LOOP);
  // tracks entry to autonomous mode
  rclcpp::Time autonomous_entry(0, 0, RCL_ROS_TIME);

  auto main_loop = [&node, &autonomous_entry, &sent_msgs_pub,
                    &current_twist_pub, &current_gps_pub, &current_imu_pub,
                    &sd_control_pub, &current_twist, &current_gps, &current_imu,
                    &sd_current_control, &battery_status_pub, &temp_battery_status,
                    &control_mode_status_pub, &current_control_mode_report,
                    &gear_status_pub, &current_gear_status,
                    &hazard_light_status_pub, &current_hazard_lights_status,
                    &indicator_status_pub, &current_indicator_status,
                    &steering_status_pub, &current_steering_status,
                    &velocity_status_pub, &current_velocity_status]() -> void {
    // Set speed source (specified at launch)
    // either NDT, IMU, or CAN bus (from vehicle sensors)
    if (ndt_speed_string == _sd_speed_source) {
      current_twist_linear_sd_mps_final = current_twist_linear_ndt_mps;
    } else if (imu_speed_string == _sd_speed_source) {
      current_twist_linear_sd_mps_final = current_twist_linear_can_imu_mps;
    } else if (vehicle_can_speed_string == _sd_speed_source) {
      current_twist_linear_sd_mps_final = current_twist_linear_can_sd_mps;
    } else {
      RCLCPP_WARN(node->get_logger(), "SD_Vehicle_Interface parameter for "
                                      "sd_speed_source is not valid\n");
    }

    // ===== UPDATE MESSAGES =====

    // angular + linear velocity
    current_twist.twist.angular.z = imu_rate_z * kDegToRad;
    current_twist.twist.linear.x =
        current_twist_linear_sd_mps_final * kUndoSDScalingFactor;
    // GPS location
    current_gps.longitude = gps_longitude;
    current_gps.latitude = gps_latitude;
    // IMU (acceleration / angular rate)
    sd::PackImuMessage(imu_variance_known_b, current_imu, imu_angle_x,
                       imu_angle_y, imu_angle_z, imu_rate_x, imu_rate_y,
                       imu_rate_z, imu_accel_x, imu_accel_y, imu_accel_z);
    current_imu.header.stamp = node->get_clock()->now();
    current_imu.header.frame_id = "imu";

    // prevent stale commands from being used
    alive_counter_z++;
    sd::update_control_alive_count(customer_control_can_tx, alive_counter_z);

    // request autonomous control if desired (at set frequency)
    if (0 == (alive_counter_z % CONTROL_LOOP)) {
      if (automation_armed_b) {
        // driver armed the vehicle for autonomous, request torque/steer
        // control
        sd::RequestAutonomousControl(customer_control_can_tx, alive_counter_z);
      } else {
        // fill CAN frame with 0s
        sd::ResetControlCanData(customer_control_can_tx, alive_counter_z);
      }
    }

    // ===== COMPUTE + PUBLISH CONTROL COMMANDS =====

    // if autonomous or simulation mode
    if (automation_granted_b || _sd_simulation_mode) {
      // run at set frequency, +0.1s delay before running calculations
      if (0 == (alive_counter_z % CONTROL_LOOP) &&
          ((node->now() - autonomous_entry) >=
           rclcpp::Duration::from_seconds(kLoopCalcDelaySec))) {

        if (is_emergency) {
          target_twist_linear_mps = 0;
          target_tire_angle_rad = 0;
        }

        // calculate steer request
        // (PID and FeedForward Contributions to Torque Controller)
        final_dbw_steer_request_pc =
            speedcontroller::CalculateSteerRequest(target_tire_angle_rad);

        // calculate torque request (different per vehicle)
        if (twizy_string == _sd_vehicle) {
          final_dbw_torque_request_pc =
              speedcontroller::CalculateTorqueRequestTwizy(
                  target_twist_linear_mps, current_twist_linear_sd_mps_final,
                  p_contribution_pc, i_contribution_pc, d_contribution_pc,
                  ff_contribution_pc);
        } else {
          final_dbw_torque_request_pc =
              speedcontroller::CalculateTorqueRequestEnv200(
                  target_twist_linear_mps, current_twist_linear_sd_mps_final,
                  p_contribution_pc, i_contribution_pc, d_contribution_pc,
                  ff_contribution_pc);
        }

        // get hazard lights request
        final_hazard_lights_request =
            auxiliarycontroller::GetHazardLightsRequest(target_hazard_lights_cmd);

        final_indicator_left_request =
            auxiliarycontroller::GetIndicatorLeftRequest(target_indicators_cmd);
        final_indicator_right_request =
            auxiliarycontroller::GetIndicatorRightRequest(target_indicators_cmd);

        // set and publish steer/torque requests
        sd_current_control.steer = final_dbw_steer_request_pc;
        sd_current_control.torque = final_dbw_torque_request_pc;
        sd_control_pub->publish(sd_current_control);
      }

      // populate Customer_Control_1 CAN frame with calculated values
      sd::PopulateControlCANData(customer_control_can_tx, final_dbw_torque_request_pc,
                            final_dbw_steer_request_pc, alive_counter_z);
      // populate Customer_Control_2
      sd::PopulateControl2CANData(customer_control_auxiliary_can_tx,
                             final_hazard_lights_request,
                             final_indicator_left_request,
                             final_indicator_right_request, alive_counter_z);
    } else { // not autonomous or simulation mode
      autonomous_entry = node->now();
    }


	// ====== CONTROL MODE OF VEHICLE ======
	using autoware_vehicle_msgs::msg::ControlModeReport;

	// using the variables taken from the can frame, create the conditions to determine the control mode
	// control mode = no command
	if (steer_automation_state == kAutoModeUnknown && torque_automation_state == kAutoModeUnknown)
		current_control_mode_report.mode = ControlModeReport::NO_COMMAND;

	// control mode = autonomous
	else if (steer_automation_state == kAutoModeAutoGranted && torque_automation_state == kAutoModeAutoGranted)
		current_control_mode_report.mode = ControlModeReport::AUTONOMOUS;

	// control mode = autonomous steer only
	else if (steer_automation_state == kAutoModeAutoGranted && torque_automation_state != kAutoModeAutoGranted)
		current_control_mode_report.mode = ControlModeReport::AUTONOMOUS_STEER_ONLY;

	// control mode = autonomous velocity only
	else if (steer_automation_state != kAutoModeAutoGranted && torque_automation_state == kAutoModeAutoGranted)
		current_control_mode_report.mode = ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;

	// control mode = manual
	else if (steer_automation_state == kAutoModeManual && torque_automation_state == kAutoModeManual)
		current_control_mode_report.mode = ControlModeReport::MANUAL;

	// control mode = disengaged
	else if ((steer_automation_state >= kAutoModeAutoSetup && steer_automation_state <= kAutoModeAutoRequested) ||
			    (torque_automation_state >= kAutoModeAutoSetup && torque_automation_state <= kAutoModeAutoRequested))
		current_control_mode_report.mode = ControlModeReport::DISENGAGED;
		
	// control mode = not ready
	else if (steer_automation_state == kAutoModeInit || steer_automation_state == kAutoModeNoCan2 || 
           steer_automation_state > kAutoModeMaxValue || torque_automation_state == kAutoModeInit || 
           torque_automation_state == kAutoModeNoCan2 || torque_automation_state > kAutoModeMaxValue)
		current_control_mode_report.mode = ControlModeReport::NOT_READY;
	
	// control mode = no command
	else
		current_control_mode_report.mode = ControlModeReport::NO_COMMAND;

    // publish to autoware (regardless on weather autonomous or not)
    temp_battery_status.stamp = node->get_clock()->now();
    battery_status_pub->publish(temp_battery_status);
    
    current_control_mode_report.stamp = node->get_clock()->now();
    control_mode_status_pub->publish(current_control_mode_report);

    current_gear_status.stamp = node->get_clock()->now();
    current_gear_status.report = target_hazard_lights_cmd;
    gear_status_pub->publish(current_gear_status);
    
    current_hazard_lights_status.stamp = node->get_clock()->now();
    current_hazard_lights_status.report = target_gear_cmd;
    hazard_light_status_pub->publish(current_hazard_lights_status);
    
    current_indicator_status.stamp = node->get_clock()->now();
    current_indicator_status.report = target_indicators_cmd;
    indicator_status_pub->publish(current_indicator_status);

    current_steering_status.stamp = node->get_clock()->now();
    current_steering_status.steering_tire_angle = (current_steer_pc * MAX_STEER_ANG) / 100;
    steering_status_pub->publish(current_steering_status);

    current_velocity_status.header.stamp = node->get_clock()->now();
    current_velocity_status.header.frame_id = "base_link"; 
    current_velocity_status.longitudinal_velocity = current_twist_linear_can_sd_mps;

    // TODO: Recheck
    static double integrated_yaw_deg = 0.0;
    integrated_yaw_deg += imu_rate_z * kYawDtSec;

    current_velocity_status.lateral_velocity = ComputeLateralVelocity(
      current_twist_linear_can_imu_mps,
      imu_angle_z,          
      integrated_yaw_deg);
    // current_velocity_status.lateral_velocity = 0

    velocity_status_pub->publish(current_velocity_status);
    
    // not simulation mode - publish control commands to vehicle
    if (!_sd_simulation_mode) {
      sent_msgs_pub->publish(customer_control_can_tx);
      sent_msgs_pub->publish(ControllerFeedbackCANTx);
    }

    // publish velocity + GPS data
    current_twist_pub->publish(current_twist);
    current_gps_pub->publish(current_gps);

    // publish IMU if one is present specified
    if (no_imu_string != _sd_gps_imu) {
      current_imu_pub->publish(current_imu);
    }
  };

  // run main loop every 5ms
  auto timer = node->create_wall_timer(std::chrono::milliseconds(kMainLoopMs), main_loop);

  try {
    // keep node active to process subscriptions, timers, services, callbacks
    rclcpp::spin(node);
  } catch (rclcpp::exceptions::RCLError &e) {
    // RCLError exception raised on Ctrl-C
    return -1;
  }

  return 0;
}
