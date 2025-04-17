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
#include <memory>
using namespace std;

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sd_auxiliary_controller.h"
#include "sd_control.h"
#include "sd_gps_imu.h"
#include "sd_lib_mcav.h"
#include "sd_msgs/msg/sd_control.hpp"
#include "sd_vehicle_interface.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <can_msgs/msg/frame.hpp>
#include <iomanip>
#include <iostream>
#include <string>

// ===== CALLBACK FUNCTIONS =====

/**
 * Process incoming CAN message, parse data into variables
 *
 * Inputs:
 *   - msg: shared pointer to received CAN frame
 */
void ReceivedFrameCANRx_callback(const shared_ptr<can_msgs::msg::Frame> msg) {

  // copy CAN frame into ReceivedFrameCANRx
  ReceivedFrameCANRx = *msg.get();

  // get current speed and automation status flags
  sd::ParseRxCANDataSDCan(ReceivedFrameCANRx, CurrentTwistLinearCANSD_Mps,
                          AutomationArmed_B, AutomationGranted_B);

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
 * extract twist angular and linear commands from autoware
 */
void AckermannCommand_callback(
    const shared_ptr<autoware_control_msgs::msg::Control> msg) {
  // Populate a twist angular and twist linear message with the received message
  // from Ros topic and convert to deg/s
  TargetTireAngle_Rad = msg->lateral.steering_tire_angle; // Radians
  TargetTwistLinear_Mps =
      msg->longitudinal.velocity / UNDO_STREETDRONE_SCALING_FACTOR; // still Mps
  TargetSteeringTireRotationRate = msg->lateral.steering_tire_rotation_rate;
}

/**
 * extract current forward speed from NDT (speed source)
 */
void CurrentVelocity_callback(
    const shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  // Current Velocity Reported from NDT
  CurrentTwistLinearNDT_Mps = msg->twist.linear.x; // mps to kph
}

/**
 * Receive hazard lights command from Ackermann and populate
 * TargetHazardLightsCmd variable
 */
void AckermannHazard_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::HazardLightsCommand> msg) {
  TargetHazardLightsCmd = msg->command;
  // testing
  cout << "TargetHazardLightsCmd: " << TargetHazardLightsCmd << "\n";
}

void AckermannIndicators_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::TurnIndicatorsCommand> msg) {
  TargetIndicatorsCmd = msg->command;
  // testing
  cout << "TargetIndicatorsCmd: " << TargetHazardLightsCmd << "\n";
}

void AckermannGear_callback(
    const shared_ptr<autoware_vehicle_msgs::msg::GearCommand> msg) {
  TargetGearCmd = msg->command;
  // testing
  cout << "TargetGearCmd: " << TargetHazardLightsCmd << "\n";
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
  sd::InitSDInterfaceControl(CustomerControlCANTx);     // Customer_Control_1
  sd::InitSDInterfaceFeedback(ControllerFeedbackCANTx); // receive feedback data
  sd::InitSDInterfaceControl2(
      CustomerControlAuxiliaryCANTx); // Customer_Control_2

  // message objects (stores incoming data)
  geometry_msgs::msg::TwistStamped current_Twist; // speed + steer
  sensor_msgs::msg::NavSatFix current_GPS;    // latitude/longitude + altitude
  sensor_msgs::msg::Imu current_IMU;          // acceleration + angular rate
  sd_msgs::msg::SDControl SD_Current_Control; // vehicle control message

  // Subscribers
  // store messages from vehicle
  auto ReceivedFrameCANRx_sub = node->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", 100, ReceivedFrameCANRx_callback);
  auto current_velocity_sub =
      node->create_subscription<geometry_msgs::msg::TwistStamped>(
          "current_velocity", 1, CurrentVelocity_callback);
  // from autoware (control commands)
  auto ackermann_cmd_sub =
      node->create_subscription<autoware_control_msgs::msg::Control>(
          "/control/command/control_cmd", 100, AckermannCommand_callback);
  // get hazard lights target from Ackermann
  auto ackerman_hazard_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", 100, AckermannHazard_callback);
  // get turn indicators target from Ackermann
  auto ackerman_indicators_sub = node->create_subscription<
      autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd	", 100,
      AckermannIndicators_callback);
  auto ackerman_gear_sub =
      node->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
          "/control/command/gear_cmd", 100, AckermannGear_callback);

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

  // set frequency of main loop (Hz)
  rclcpp::Rate loop_rate(ROS_LOOP);
  // tracks entry to autonomous mode
  rclcpp::Time autonomous_entry(0, 0, RCL_ROS_TIME);

  auto main_loop = [&node, &autonomous_entry, &sent_msgs_pub,
                    &current_twist_pub, &current_GPS_pub, &current_IMU_pub,
                    &sd_control_pub, &current_Twist, &current_GPS, &current_IMU,
                    &SD_Current_Control]() -> void {
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
    sd::UpdateControlAlive(CustomerControlCANTx, AliveCounter_Z);

    // request autonomous control if desired (at set frequency)
    if (0 == (AliveCounter_Z % CONTROL_LOOP)) {
      if (AutomationArmed_B) {
        // driver armed the vehicle for autonomous, request torque/steer
        // control
        sd::RequestAutonomousControl(CustomerControlCANTx, AliveCounter_Z);
      } else {
        // fill CAN frame with 0s
        sd::ResetControlCanData(CustomerControlCANTx, AliveCounter_Z);
      }
    }

    // ===== COMPUTE + PUBLISH CONTROL COMMANDS =====

    // if autonomous or simulation mode
    if (AutomationGranted_B || _sd_simulation_mode) {
      // run at set frequency, +0.1s delay before running calculations
      if (0 == (AliveCounter_Z % CONTROL_LOOP) &&
          ((node->now() - autonomous_entry) >=
           rclcpp::Duration::from_seconds(0.1))) {

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
      sd::PopControlCANData(CustomerControlCANTx, FinalDBWTorqueRequest_Pc,
                            FinalDBWSteerRequest_Pc, AliveCounter_Z);
      // populate Customer_Control_2
      sd::PopControl2CANData(CustomerControlAuxiliaryCANTx,
                             FinalHazardLightsRequest,
                             FinalIndicatorLeftRequest,
                             FinalIndicatorRightRequest, AliveCounter_Z);
    } else { // not autonomous or simulation mode
      autonomous_entry = node->now();
    }

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
