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
#include "sd_vehicle_interface.h"  // NOLINT(build/include_subdir)
#include <memory>
#include <string>
#include <can_msgs/msg/frame.hpp>
#include "autoware_control_msgs/msg/control.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sd_msgs/msg/sd_control.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sd_control.h"  // NOLINT(build/include_subdir)
#include "sd_gps_imu.h"  // NOLINT(build/include_subdir)
#include "sd_lib_mcav.h"  // NOLINT(build/include_subdir)

// Callback functions
void ReceivedFrameCANRx_callback(const std::shared_ptr<can_msgs::msg::Frame> msg)
{
  // Populates into ReceivedFrameCANRx the latest can message
  ReceivedFrameCANRx = *msg.get();
  sd::ParseRxCANDataSDCan(
    ReceivedFrameCANRx, CurrentTwistLinearCANSD_Mps, AutomationArmed_B,
    AutomationGranted_B);
  if (oxts_string == _sd_gps_imu) {
    IMUVarianceKnown_B = true;  // Variance/covariance known for OXTS.
    // Use the OXTS parsing function.
    sd::ParseRxCANDataOXTSCan(
      ReceivedFrameCANRx, CurrentTwistLinearCANImu_Mps, GPS_Longitude,
      GPS_Latitude, IMU_Angle_X, IMU_Angle_Y, IMU_Angle_Z, IMU_Rate_X,
      IMU_Rate_Y, IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
  } else if (peak_string == _sd_gps_imu) {
    // Use the PEAK parsing function.
    IMUVarianceKnown_B = false;  // Variance/covariance not known for PEAK.
    sd::ParseRxCANDataPEAKCan(
      ReceivedFrameCANRx, CurrentTwistLinearCANImu_Mps, GPS_Longitude,
      GPS_Latitude, IMU_Angle_X, IMU_Angle_Y, IMU_Angle_Z, IMU_Rate_X,
      IMU_Rate_Y, IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
  } else if (no_imu_string == _sd_gps_imu) {
    // Do nothing.
  } else {
    // RCLCPP_WARN(node->get_logger(), "SD_Vehicle_Interface parameter for sd_gps_imu is not valid\n");
  }
}


void AckermannCommand_callback(
  const std::shared_ptr<autoware_control_msgs::msg::Control> msg)
{
  // Populate steering/twist targets from incoming control command.
  TargeTireAngle_Rad = msg->lateral.steering_tire_angle;  // Radians.
  TargetTwistLinear_Mps =
    msg->longitudinal.velocity / UNDO_STREETDRONE_SCALING_FACTOR;  // M/s.
}

void CurrentVelocity_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg)
{
  // Current velocity reported from NDT.
  CurrentTwistLinearNDT_Mps = msg->twist.linear.x;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sd_twizy_interface_node");
  node->declare_parameter<std::string>("sd_vehicle", "env200");
  _sd_vehicle = node->get_parameter("sd_vehicle").as_string();
  node->declare_parameter<std::string>("sd_gps_imu", "oxts");
  _sd_gps_imu = node->get_parameter("sd_gps_imu").as_string();
  node->declare_parameter<std::string>("sd_speed_source", "vehicle_can_speed");
  _sd_speed_source = node->get_parameter("sd_speed_source").as_string();
  node->declare_parameter<bool>("sd_simulation_mode", false);
  _sd_simulation_mode = node->get_parameter("sd_simulation_mode").as_bool();

  // Initialise StreetDrone output CAN variables.
  sd::InitSDInterfaceControl(CustomerControlCANTx);
  sd::InitSDInterfaceFeedback(ControllerFeedbackCANTx);
  geometry_msgs::msg::TwistStamped current_Twist;
  sensor_msgs::msg::NavSatFix current_GPS;
  sensor_msgs::msg::Imu current_IMU;
  sd_msgs::msg::SDControl SD_Current_Control;

  // Subscriber
  auto ReceivedFrameCANRx_sub = node->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 100,
    ReceivedFrameCANRx_callback);
  auto current_velocity_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "current_velocity", 1, CurrentVelocity_callback);
  auto ackermann_cmd_sub = node->create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 100, AckermannCommand_callback);

  // Publisher
  auto sent_msgs_pub = node->create_publisher<can_msgs::msg::Frame>("to_can_bus", 100);
  auto current_twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "sd_current_twist", 100);
  auto current_GPS_pub =
    node->create_publisher<sensor_msgs::msg::NavSatFix>("sd_current_GPS", 100);
  auto current_IMU_pub = node->create_publisher<sensor_msgs::msg::Imu>("sd_imu_raw", 100);
  auto sd_control_pub =
    node->create_publisher<sd_msgs::msg::SDControl>("sd_control", 1);

  rclcpp::Rate loop_rate(ROS_LOOP);
  rclcpp::Time autonomous_entry(0, 0, RCL_ROS_TIME);

  auto main_loop =
    [&node, &autonomous_entry, &sent_msgs_pub, &current_twist_pub, &current_GPS_pub,
      &current_IMU_pub, &sd_control_pub,
      &current_Twist, &current_GPS, &current_IMU, &SD_Current_Control]() -> void
    {
      // Choose vehicle speed source specified at launch.
      if (ndt_speed_string == _sd_speed_source) {
        CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearNDT_Mps;
      } else if (imu_speed_string == _sd_speed_source) {
        CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearCANImu_Mps;
      } else if (vehicle_can_speed_string == _sd_speed_source) {
        CurrentTwistLinearSD_Mps_Final = CurrentTwistLinearCANSD_Mps;
      } else {
        RCLCPP_WARN(
          node->get_logger(), "SD_Vehicle_Interface parameter for sd_speed_source is not valid\n");
      }

      current_Twist.twist.angular.z = IMU_Rate_Z * DEG_to_RAD;
      current_Twist.twist.linear.x = CurrentTwistLinearSD_Mps_Final *
        UNDO_STREETDRONE_SCALING_FACTOR;
      // Prepare GPS message with latest data.
      current_GPS.longitude = GPS_Longitude;
      current_GPS.latitude = GPS_Latitude;

      // Prepare IMU message with latest data.
      sd::PackImuMessage(
        IMUVarianceKnown_B, current_IMU, IMU_Angle_X, IMU_Angle_Y, IMU_Angle_Z,
        IMU_Rate_X, IMU_Rate_Y, IMU_Rate_Z, IMU_Accel_X, IMU_Accel_Y, IMU_Accel_Z);
      current_IMU.header.stamp = node->get_clock()->now();
      current_IMU.header.frame_id = "imu";

      // Prepare TX CAN messages with latest data.
      AliveCounter_Z++;  // Increment the alive counter.

      sd::UpdateControlAlive(CustomerControlCANTx, AliveCounter_Z);

      if (0 == (AliveCounter_Z % CONTROL_LOOP)) {
        if (AutomationArmed_B) {
          sd::RequestAutonomousControl(CustomerControlCANTx, AliveCounter_Z);
        } else {
          sd::ResetControlCanData(CustomerControlCANTx, AliveCounter_Z);
        }
      }

      if (AutomationGranted_B || _sd_simulation_mode) {
        if (
          0 == (AliveCounter_Z % CONTROL_LOOP) &&
          ((node->now() - autonomous_entry) >= rclcpp::Duration::from_seconds(0.1)))
        {
          // Calculate steer/torque and feedback contributions.
          FinalDBWSteerRequest_Pc = speedcontroller::CalculateSteerRequest(TargeTireAngle_Rad);

          if (twizy_string == _sd_vehicle) {
            FinalDBWTorqueRequest_Pc = speedcontroller::CalculateTorqueRequestTwizy(
              TargetTwistLinear_Mps, CurrentTwistLinearSD_Mps_Final, P_Contribution_Pc,
              I_Contribution_Pc, D_Contribution_Pc, FF_Contribution_Pc);
          } else {
            FinalDBWTorqueRequest_Pc = speedcontroller::CalculateTorqueRequestEnv200(
              TargetTwistLinear_Mps, CurrentTwistLinearSD_Mps_Final, P_Contribution_Pc,
              I_Contribution_Pc, D_Contribution_Pc, FF_Contribution_Pc);
          }

          SD_Current_Control.steer = FinalDBWSteerRequest_Pc;
          SD_Current_Control.torque = FinalDBWTorqueRequest_Pc;
          sd_control_pub->publish(SD_Current_Control);
        }

        // Populate CAN frames with calculated data.
        sd::PopControlCANData(
          CustomerControlCANTx, FinalDBWTorqueRequest_Pc,
          FinalDBWSteerRequest_Pc, AliveCounter_Z);
      } else {
        autonomous_entry = node->now();
      }

      // If not in simulation mode, publish control/feedback CAN frames.
      if (!_sd_simulation_mode) {
        sent_msgs_pub->publish(CustomerControlCANTx);
        sent_msgs_pub->publish(ControllerFeedbackCANTx);
      }

      current_twist_pub->publish(current_Twist);
      current_GPS_pub->publish(current_GPS);

      // If an IMU source is configured, publish IMU message.
      if (no_imu_string != _sd_gps_imu) {
        current_IMU_pub->publish(current_IMU);
      }
    };

  auto timer = node->create_wall_timer(std::chrono::milliseconds(5), main_loop);  // 5ms gives 200Hz loop rate

  try {
    rclcpp::spin(node);
  } catch (rclcpp::exceptions::RCLError & e) {
    // RCLError exception raised on Ctrl-C
    return -1;
  }

  return 0;
}
