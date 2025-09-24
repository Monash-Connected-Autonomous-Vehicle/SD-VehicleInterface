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

using namespace std;
using autoware_vehicle_msgs::msg::ControlModeReport;

// Stating variables for automation modes
uint8_t Steer_Automation_State  = 0;
uint8_t Torque_Automation_State = 0;

// ===== CALLBACK FUNCTIONS =====

/**
 * Process incoming CAN message, parse data into variables
 *
 * Inputs:
 *   - msg: shared pointer to received CAN frame
 */
void on_can_rx_frame(const std::shared_ptr<can_msgs::msg::Frame> msg) {

  // copy CAN frame into ReceivedFrameCANRx
  ReceivedFrameCANRx = *msg.get();

  // get current speed, automation status flags and autonomation states
  sd::parse_sd_can_rx_frame(ReceivedFrameCANRx, CurrentTwistLinearCANSD_Mps, CurrentSteer_pc,
                          AutomationArmed_B, AutomationGranted_B, Steer_Automation_State, Torque_Automation_State);

  // parse data depending on IMU/GPS device used
  if (oxts_string == _sd_gps_imu) {
    IMUVarianceKnown_B = true; // variance/covariance known for OXTS
    // parse using OXTS function
    sd::ParseRxCANDataOXTSCan(
        ReceivedFrameCANRx, CurrentTwistLinearCANImu_Mps, GPS_Longitude,
        GPS_Latitude, IMU_Angle_X, IMU_Angle_Y, IMU_Angle_Z, IMU_Rate_X,
        IMU_Rate_Y, IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
  } else if (peak_string == _sd_gps_imu) {
    // parse using PEAK function
    IMUVarianceKnown_B = false; // variance/covariance not known for PEAK
    sd::ParseRxCANDataPEAKCan(
        ReceivedFrameCANRx, CurrentTwistLinearCANImu_Mps, GPS_Longitude,
        GPS_Latitude, IMU_Angle_X, IMU_Angle_Y, IMU_Angle_Z, IMU_Rate_X,
        IMU_Rate_Y, IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
  } else if (no_imu_string == _sd_gps_imu) {
    // no IMU, do nothing
  } else {
    // unkonwn IMU, warning
    // RCLCPP_WARN(node->get_logger(), "SD_Vehicle_Interface parameter for
    // sd_gps_imu is not valid\n");
  }
}

/**
 * extract current forward speed from NDT (speed source)
 */
void current_velocity_callback(
    const shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  // Current Velocity Reported from NDT
  CurrentTwistLinearNDT_Mps = msg->twist.linear.x; // mps to kph
}

// ===== AUTOWARE PUB/SUB =====
void control_cmd_callback(
    const shared_ptr<autoware_control_msgs::msg::Control> msg) {
  TargetTireAngle_Rad = msg->lateral.steering_tire_angle;
  TargetTwistLinear_Mps =
      msg->longitudinal.velocity / UNDO_STREETDRONE_SCALING_FACTOR;
  TargetSteeringTireRotationRate = msg->lateral.steering_tire_rotation_rate;
}

void hazard_cmd_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> msg) {
  TargetHazardLightsCmd = msg->command;
}

void indicators_cmd_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::TurnIndicatorsCommand> msg) {
  TargetIndicatorsCmd = msg->command;
}

void gear_cmd_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::GearCommand> msg) {
  TargetGearCmd = msg->command;
}

void gate_mode_cmd_callback(
    const shared_ptr<tier4_control_msgs::msg::GateMode> msg) {
  TargetGateModeCmd = msg->data;
}

void emergency_cmd_callback(
    const shared_ptr<tier4_vehicle_msgs::msg::VehicleEmergencyStamped> msg) {
  IsEmergency = msg->emergency;
}

void actuation_cmd_callback(
    const shared_ptr<tier4_vehicle_msgs::msg::ActuationCommandStamped> msg) {
  TargetAccelCmd_temp = msg->actuation.accel_cmd;
  TargetBrakeCmd_temp = msg->actuation.brake_cmd;
  TargetSteerCmd_temp = msg->actuation.steer_cmd;
}

double compute_lateral_velocity(double vehicle_speed_mps,
                              double gps_course_deg,
                              double integrated_yaw_deg) {
  constexpr double DEG_TO_RAD = M_PI / 180.0;
  double course_rad = gps_course_deg * DEG_TO_RAD;
  double yaw_rad = integrated_yaw_deg * DEG_TO_RAD;

  return vehicle_speed_mps * std::sin(course_rad - yaw_rad);
}


// ===== MAIN FUNCTION =====

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  // ===== INITIALISE REQUIRED OBJECTS =====

  // create node + declare parameters:
  //   "sd_vehicle", "sd_gps_imu", "sd_speed_source", "sd_simulation_mode"
  auto node = rclcpp::Node::make_shared("sd_twizy_interface_node");
  node->declare_parameter<string>("sd_vehicle", "env200");
  _sd_vehicle = node->get_parameter("sd_vehicle").as_string();
  node->declare_parameter<string>("sd_gps_imu", "oxts");
  _sd_gps_imu = node->get_parameter("sd_gps_imu").as_string();
  node->declare_parameter<string>("sd_speed_source", "vehicle_can_speed");
  _sd_speed_source = node->get_parameter("sd_speed_source").as_string();
  node->declare_parameter<bool>("sd_simulation_mode", false);
  _sd_simulation_mode = node->get_parameter("sd_simulation_mode").as_bool();

  // initialise CAN variables
  sd::initialise_sd_interface_control(CustomerControlCANTx);     // Customer_Control_1
  sd::initialise_sd_interface_feedback(ControllerFeedbackCANTx); // receive feedback data
  sd::initialise_sd_interface_control_2(
      CustomerControlAuxiliaryCANTx); // Customer_Control_2

  // message objects (stores incoming data)
  geometry_msgs::msg::TwistStamped current_Twist; // speed + steer
  sensor_msgs::msg::NavSatFix current_GPS;    // latitude/longitude + altitude
  sensor_msgs::msg::Imu current_IMU;          // acceleration + angular rate
  sd_msgs::msg::SDControl SD_Current_Control; // vehicle control message

  // Subscribers
  // store messages from vehicle
  auto ReceivedFrameCANRx_sub = node->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", 100, on_can_rx_frame);
  auto current_velocity_sub =
      node->create_subscription<geometry_msgs::msg::TwistStamped>(
          "current_velocity", 1, current_velocity_callback);
  
  // Autoware-specific subscribers
  auto control_sub =
      node->create_subscription<autoware_control_msgs::msg::Control>(
          "/control/command/control_cmd", 100, control_cmd_callback);
  auto gear_sub =
      node->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
          "/control/command/gear_cmd", 100, gear_cmd_callback);
  auto gate_mode_sub =
      node->create_subscription<tier4_control_msgs::msg::GateMode>(
          "/control/current_gate_mode", 100, gate_mode_cmd_callback);
  auto emergency_cmd_sub =
      node->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
          "/control/command/emergency_cmd", 100, emergency_cmd_callback);
  auto actuation_cmd_sub =
      node->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
          "/control/command/actuation_cmd", 100, actuation_cmd_callback);
  auto hazard_lights_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", 100, hazard_cmd_callback);
  auto indicators_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", 100,
      indicators_cmd_callback);

  // Publishers
  // control commands (for ENV200)
  auto sent_msgs_pub =
      node->create_publisher<can_msgs::msg::Frame>("to_can_bus", 100);

  // current velocity
  auto current_twist_pub =
      node->create_publisher<geometry_msgs::msg::TwistStamped>(
          "sd_current_twist", 100);
  auto current_GPS_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(
      "sd_current_GPS", 100);
  auto current_IMU_pub =
      node->create_publisher<sensor_msgs::msg::Imu>("sd_imu_raw", 100);
  auto sd_control_pub =
      node->create_publisher<sd_msgs::msg::SDControl>("sd_control", 1);
  
  // Autoware-specific publishers and message stores
  tier4_vehicle_msgs::msg::BatteryStatus temp_BatteryStatus;
  temp_BatteryStatus.energy_level = 100;
  auto battery_status_pub =
      node->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("vehicle/status/battery_charge", 10);

  autoware_vehicle_msgs::msg::ControlModeReport current_ControlModeReport;
  current_ControlModeReport.mode = ControlModeReport::AUTONOMOUS; 
  auto control_mode_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("vehicle/status/control_mode", 10);
  
  autoware_vehicle_msgs::msg::GearReport current_GearStatus;
  auto gear_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::GearReport>("vehicle/status/gear_status", 10);
  
  autoware_vehicle_msgs::msg::HazardLightsReport current_HazardLightsStatus;
  auto hazard_light_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("vehicle/status/hazard_lights_status", 10);

  autoware_vehicle_msgs::msg::TurnIndicatorsReport current_IndicatorStatus;
  auto indicator_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("vehicle/status/turn_indicators_status", 10);

  autoware_vehicle_msgs::msg::SteeringReport current_SteeringStatus;
  auto steering_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("vehicle/status/steering_status", 10);

  autoware_vehicle_msgs::msg::VelocityReport current_VelocityStatus;
  auto velocity_status_pub =
      node->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("vehicle/status/velocity_status", 10);


  // set frequency of main loop (Hz)
  rclcpp::Rate loop_rate(ROS_LOOP);
  // tracks entry to autonomous mode
  rclcpp::Time autonomous_entry(0, 0, RCL_ROS_TIME);

  auto main_loop = [&node, &autonomous_entry, &sent_msgs_pub,
                    &current_twist_pub, &current_GPS_pub, &current_IMU_pub,
                    &sd_control_pub, &current_Twist, &current_GPS, &current_IMU,
                    &SD_Current_Control, &battery_status_pub, &temp_BatteryStatus,
                    &control_mode_status_pub, &current_ControlModeReport,
                    &gear_status_pub, &current_GearStatus,
                    &hazard_light_status_pub, &current_HazardLightsStatus,
                    &indicator_status_pub, &current_IndicatorStatus,
                    &steering_status_pub, &current_SteeringStatus,
                    &velocity_status_pub, &current_VelocityStatus]() -> void {
    // Set speed source (specified at launch)
    // either NDT, IMU, or CAN bus (from vehicle sensors)
    if (ndt_speed_string == _sd_speed_source) {
      CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearNDT_Mps;
    } else if (imu_speed_string == _sd_speed_source) {
      CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearCANImu_Mps;
    } else if (vehicle_can_speed_string == _sd_speed_source) {
      CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearCANSD_Mps;
    } else {
      RCLCPP_WARN(node->get_logger(), "SD_Vehicle_Interface parameter for "
                                      "sd_speed_source is not valid\n");
    }

    // ===== UPDATE MESSAGES =====

    // angular + linear velocity
    current_Twist.twist.angular.z = IMU_Rate_Z * DEG_to_RAD;
    current_Twist.twist.linear.x =
        CurrentTwistLinearSD_Mps_Final * UNDO_STREETDRONE_SCALING_FACTOR;
    // GPS location
    current_GPS.longitude = GPS_Longitude;
    current_GPS.latitude = GPS_Latitude;
    // IMU (acceleration / angular rate)
    sd::PackImuMessage(IMUVarianceKnown_B, current_IMU, IMU_Angle_X,
                       IMU_Angle_Y, IMU_Angle_Z, IMU_Rate_X, IMU_Rate_Y,
                       IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
    current_IMU.header.stamp = node->get_clock()->now();
    current_IMU.header.frame_id = "imu";

    // prevent stale commands from being used
    AliveCounter_Z++;
    sd::update_control_alive_count(CustomerControlCANTx, AliveCounter_Z);

    // request autonomous control if desired (at set frequency)
    if (0 == (AliveCounter_Z % CONTROL_LOOP)) {
      if (AutomationArmed_B) {
        // driver armed the vehicle for autonomous, request torque/steer
        // control
        sd::request_autonomous_control(CustomerControlCANTx, AliveCounter_Z);
      } else {
        // fill CAN frame with 0s
        sd::reset_control_can_data(CustomerControlCANTx, AliveCounter_Z);
      }
    }

    // ===== COMPUTE + PUBLISH CONTROL COMMANDS =====

    // if autonomous or simulation mode
    if (AutomationGranted_B || _sd_simulation_mode) {
      // run at set frequency, +0.1s delay before running calculations
      if (0 == (AliveCounter_Z % CONTROL_LOOP) &&
          ((node->now() - autonomous_entry) >=
           rclcpp::Duration::from_seconds(0.1))) {

        if (IsEmergency) {
          TargetTwistLinear_Mps = 0;
          TargetTireAngle_Rad = 0;
        }

        // calculate steer request
        // (PID and FeedForward Contributions to Torque Controller)
        FinalDBWSteerRequest_Pc =
            speedcontroller::CalculateSteerRequest(TargetTireAngle_Rad);

        // calculate torque request (different per vehicle)
        if (twizy_string == _sd_vehicle) {
          FinalDBWTorqueRequest_Pc =
              speedcontroller::CalculateTorqueRequestTwizy(
                  TargetTwistLinear_Mps, CurrentTwistLinearSD_Mps_Final,
                  P_Contribution_Pc, I_Contribution_Pc, D_Contribution_Pc,
                  FF_Contribution_Pc);
        } else {
          FinalDBWTorqueRequest_Pc =
              speedcontroller::CalculateTorqueRequestEnv200(
                  TargetTwistLinear_Mps, CurrentTwistLinearSD_Mps_Final,
                  P_Contribution_Pc, I_Contribution_Pc, D_Contribution_Pc,
                  FF_Contribution_Pc);
        }

        // get hazard lights request
        FinalHazardLightsRequest =
            auxiliarycontroller::GetHazardLightsRequest(TargetHazardLightsCmd);

        FinalIndicatorLeftRequest =
            auxiliarycontroller::GetIndicatorLeftRequest(TargetIndicatorsCmd);
        FinalIndicatorRightRequest =
            auxiliarycontroller::GetIndicatorRightRequest(TargetIndicatorsCmd);

        // set and publish steer/torque requests
        SD_Current_Control.steer = FinalDBWSteerRequest_Pc;
        SD_Current_Control.torque = FinalDBWTorqueRequest_Pc;
        sd_control_pub->publish(SD_Current_Control);
      }

      // populate Customer_Control_1 CAN frame with calculated values
      sd::populate_control_can_data(CustomerControlCANTx, FinalDBWTorqueRequest_Pc,
                            FinalDBWSteerRequest_Pc, AliveCounter_Z);
      // populate Customer_Control_2
      sd::populate_control_2_can_data(CustomerControlAuxiliaryCANTx,
                             FinalHazardLightsRequest,
                             FinalIndicatorLeftRequest,
                             FinalIndicatorRightRequest, AliveCounter_Z);
    } else { // not autonomous or simulation mode
      autonomous_entry = node->now();
    }


	// ====== CONTROL MODE OF VEHICLE ======
	using autoware_vehicle_msgs::msg::ControlModeReport;

	// using the variables taken from the can frame, create the conditions to determine the control mode
	// control mode = no command
	if (Steer_Automation_State == 0 && Torque_Automation_State == 0)
		current_ControlModeReport.mode = ControlModeReport::NO_COMMAND;

	// control mode = autonomous
	else if (Steer_Automation_State == 7 && Torque_Automation_State == 7)
		current_ControlModeReport.mode = ControlModeReport::AUTONOMOUS;

	// control mode = autonomous steer only
	else if (Steer_Automation_State == 7 && Torque_Automation_State != 7)
		current_ControlModeReport.mode = ControlModeReport::AUTONOMOUS_STEER_ONLY;

	// control mode = autonomous velocity only
	else if (Steer_Automation_State != 7 && Torque_Automation_State == 7)
		current_ControlModeReport.mode = ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;

	// control mode = manual
	else if (Steer_Automation_State == 3 && Torque_Automation_State == 3)
		current_ControlModeReport.mode = ControlModeReport::MANUAL;

	// control mode = disengaged
	else if ((Steer_Automation_State >= 4 && Steer_Automation_State <= 6) ||
			(Torque_Automation_State >= 4 && Torque_Automation_State <= 6))
		current_ControlModeReport.mode = ControlModeReport::DISENGAGED;
		
	// control mode = not ready
	else if (Steer_Automation_State == 1 || Steer_Automation_State == 2 || Steer_Automation_State > 8 ||
			Torque_Automation_State == 1 || Torque_Automation_State == 2 || Torque_Automation_State > 8)
		current_ControlModeReport.mode = ControlModeReport::NOT_READY;
	
	// control mode = no command
	else
		current_ControlModeReport.mode = ControlModeReport::NO_COMMAND;



    // publish to autoware (regardless on weather autonomous or not)
    temp_BatteryStatus.stamp = node->get_clock()->now();
    battery_status_pub->publish(temp_BatteryStatus);
    
    current_ControlModeReport.stamp = node->get_clock()->now();
    control_mode_status_pub->publish(current_ControlModeReport);

    current_GearStatus.stamp = node->get_clock()->now();
    current_GearStatus.report = TargetHazardLightsCmd;
    gear_status_pub->publish(current_GearStatus);
    
    current_HazardLightsStatus.stamp = node->get_clock()->now();
    current_HazardLightsStatus.report = TargetGearCmd;
    hazard_light_status_pub->publish(current_HazardLightsStatus);
    
    current_IndicatorStatus.stamp = node->get_clock()->now();
    current_IndicatorStatus.report = TargetIndicatorsCmd;
    indicator_status_pub->publish(current_IndicatorStatus);

    current_SteeringStatus.stamp = node->get_clock()->now();
    current_SteeringStatus.steering_tire_angle = CurrentSteer_pc * MAX_STEER_ANG;
    steering_status_pub->publish(current_SteeringStatus);

    current_VelocityStatus.header.stamp = node->get_clock()->now();
    current_VelocityStatus.header.frame_id = "base_link"; 
    current_VelocityStatus.longitudinal_velocity = CurrentTwistLinearCANSD_Mps;

    // TODO: Recheck
    static double integrated_yaw_deg = 0.0;
    constexpr double dt = 0.05;
    integrated_yaw_deg += IMU_Rate_Z * dt;

    current_VelocityStatus.lateral_velocity = compute_lateral_velocity(
      CurrentTwistLinearCANImu_Mps,
      IMU_Angle_Z,          
      integrated_yaw_deg);
    // current_VelocityStatus.lateral_velocity = 0

    velocity_status_pub->publish(current_VelocityStatus);
    
    // not simulation mode - publish control commands to vehicle
    if (!_sd_simulation_mode) {
      sent_msgs_pub->publish(CustomerControlCANTx);
      sent_msgs_pub->publish(ControllerFeedbackCANTx);
    }

    // publish velocity + GPS data
    current_twist_pub->publish(current_Twist);
    current_GPS_pub->publish(current_GPS);

    // publish IMU if one is present specified
    if (no_imu_string != _sd_gps_imu) {
      current_IMU_pub->publish(current_IMU);
    }
  };

  // run main loop every 5ms
  auto timer = node->create_wall_timer(5ms, main_loop);

  try {
    // keep node active to process subscriptions, timers, services, callbacks
    rclcpp::spin(node);
  } catch (rclcpp::exceptions::RCLError &e) {
    // RCLError exception raised on Ctrl-C
    return -1;
  }

  return 0;
}
