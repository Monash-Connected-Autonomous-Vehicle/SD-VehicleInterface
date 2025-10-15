/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 *
 * Author: Fionán O'Sullivan
 *
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

#include "sd_gps_imu.h"
#include <cmath>

namespace sd {

// Functions

// Converts from Euler to Quaternion format.
Quaternion ToQuaternion(double yaw, double pitch,
                        double roll) // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

void ParseRxCanDataOxtsCan(can_msgs::msg::Frame &ReceivedFrameCAN,
                           double &CurrentLinearVelocity_Mps,
                           double &gps_longitude, double &gps_latitude,
                           double &imu_angle_x, double &imu_angle_y,
                           double &imu_angle_z, double &imu_rate_x,
                           double &imu_rate_y, double &imu_rate_z,
                           double &imu_accel_x, double &imu_accel_y,
                           double &imu_accel_z)

// This function parses the received can traffic from the OXTS unit. Please
// see OXTS documentation
{
  CAN_frame_t ReceivedFrameUnion;
  ReceivedFrameUnion.byte[0] = ReceivedFrameCAN.data[0];
  ReceivedFrameUnion.byte[1] = ReceivedFrameCAN.data[1];
  ReceivedFrameUnion.byte[2] = ReceivedFrameCAN.data[2];
  ReceivedFrameUnion.byte[3] = ReceivedFrameCAN.data[3];
  ReceivedFrameUnion.byte[4] = ReceivedFrameCAN.data[4];
  ReceivedFrameUnion.byte[5] = ReceivedFrameCAN.data[5];
  ReceivedFrameUnion.byte[6] = ReceivedFrameCAN.data[6];
  ReceivedFrameUnion.byte[7] = ReceivedFrameCAN.data[7];

  switch (ReceivedFrameCAN.id) {
  case 1539: // 0x603
  {
    CurrentLinearVelocity_Mps = ReceivedFrameUnion.word[3] * 0.01;

    break;
  }
  case 1537: // 0x601
  {
    gps_longitude = ReceivedFrameUnion.dword[1] * 0.0000001;
    gps_latitude = ReceivedFrameUnion.dword[0] * 0.0000001;

    break;
  }
  case 1541: // 0x605
  {
    imu_accel_x = ReceivedFrameUnion.word[0] * 0.01;
    imu_accel_y = ReceivedFrameUnion.word[1] * 0.01;
    imu_accel_z = ReceivedFrameUnion.word[2] * 0.01;

    break;
  }
  case 1543: // 0x607
  {
    imu_angle_x = ReceivedFrameUnion.word[0] * 0.01;
    imu_angle_y = ReceivedFrameUnion.word[1] * 0.01;
    imu_angle_z = ReceivedFrameUnion.word[2] * 0.01;

    break;
  }
  case 1544: // 0x608
  {
    imu_rate_x = ReceivedFrameUnion.word[0] * 0.01;
    imu_rate_y = ReceivedFrameUnion.word[1] * 0.01;
    imu_rate_z = ReceivedFrameUnion.word[2] * 0.01;

    break;
  }
  }
}

void ParseRxCanDataPeakCan(can_msgs::msg::Frame &ReceivedFrameCAN,
                           double &CurrentLinearVelocity_Mps,
                           double &gps_longitude, double &gps_latitude,
                           double &imu_angle_x, double &imu_angle_y,
                           double &imu_angle_z, double &imu_rate_x,
                           double &imu_rate_y, double &imu_rate_z,
                           double &imu_accel_x, double &imu_accel_y,
                           double &imu_accel_z)

// This function parses the received can traffic from the OXTS unit. Please
// see OXTS documentation
{
  CAN_frame_t ReceivedFrameUnion;
  ReceivedFrameUnion.byte[0] = ReceivedFrameCAN.data[0];
  ReceivedFrameUnion.byte[1] = ReceivedFrameCAN.data[1];
  ReceivedFrameUnion.byte[2] = ReceivedFrameCAN.data[2];
  ReceivedFrameUnion.byte[3] = ReceivedFrameCAN.data[3];
  ReceivedFrameUnion.byte[4] = ReceivedFrameCAN.data[4];
  ReceivedFrameUnion.byte[5] = ReceivedFrameCAN.data[5];
  ReceivedFrameUnion.byte[6] = ReceivedFrameCAN.data[6];
  ReceivedFrameUnion.byte[7] = ReceivedFrameCAN.data[7];

  switch (ReceivedFrameCAN.id) {
  case 1536: // 0x600
  {
    imu_accel_x =
        ((int16_t)ReceivedFrameUnion.word[0]) * 3.91 * 0.001 *
        9.80665; // 0.038344002; //*3.91 = mG, then conversion to m/s^2
    imu_accel_y =
        ((int16_t)ReceivedFrameUnion.word[1]) * 3.91 * 0.001 * 9.80665; //
    imu_accel_z =
        ((int16_t)ReceivedFrameUnion.word[2]) * 3.91 * 0.001 * 9.80665; //

    break;
  }
  case 1552: // 0x610
  {
    float_bits_converter imu_rate_x_fbc;
    float_bits_converter imu_rate_y_fbc;

    imu_rate_x_fbc.integer_can = ReceivedFrameUnion.dword[0];
    imu_rate_y_fbc.integer_can = ReceivedFrameUnion.dword[1];
    imu_rate_x = imu_rate_x_fbc.float_can;
    imu_rate_y = imu_rate_y_fbc.float_can;

    imu_angle_x +=
        imu_rate_x *
        0.05; // Angle achieved by integrating rate over time (50 ms rate)
    imu_angle_y +=
        imu_rate_y *
        0.05; // Angle achieved by integrating rate over time (50 ms rate)

    break;
  }
  case 1553: // 0x611
  {
    float_bits_converter imu_rate_z_fbc;

    imu_rate_z_fbc.integer_can = ReceivedFrameUnion.dword[0];
    imu_rate_z = imu_rate_z_fbc.float_can;
    imu_angle_z +=
        imu_rate_z *
        0.05; // Angle achieved by integrating rate over time (50 ms rate)
    break;
  }

  case 1569: // 0x621
  {
    float_bits_converter CurrentLinearVelocity_Mps_fbc;
    float_bits_converter Course_Deg_fbc;

    CurrentLinearVelocity_Mps_fbc.integer_can = ReceivedFrameUnion.dword[1];
    CurrentLinearVelocity_Mps =
        CurrentLinearVelocity_Mps_fbc.float_can * KPH_to_MPS;

    Course_Deg_fbc.integer_can = ReceivedFrameUnion.dword[0];
    imu_angle_z = Course_Deg_fbc.float_can; // This is a better measurement
                                            // than integrated rate over time.

    break;
  }
  case 1571: // 0x623
  {
    float_bits_converter gps_latitude_Minutes_fbc;
    gps_latitude_Minutes_fbc.integer_can = ReceivedFrameUnion.dword[0];

    gps_latitude =
        (gps_latitude_Minutes_fbc.float_can / 60) + ReceivedFrameUnion.word[2];

    // As per PEAK CAN GPS .dbc, 83 represents south.
    if (ReceivedFrameUnion.byte[6] == 83) {
      gps_latitude *= -1;
    }
    break;
  }
  case 1570: // 0x623
  {
    float_bits_converter gps_longitude_Minutes_fbc;
    gps_longitude_Minutes_fbc.integer_can = ReceivedFrameUnion.dword[0];

    gps_longitude =
        (gps_longitude_Minutes_fbc.float_can / 60) + ReceivedFrameUnion.word[2];

    // As per PEAK CAN GPS .dbc, 87 represents west.
    if (ReceivedFrameUnion.byte[6] == 87) {
      gps_longitude *= -1;
    }
    break;
  }
  }
}

void PackImuMessage(bool imu_variance_known_b, sensor_msgs::msg::Imu &current_imu,
                    double imu_angle_x, double imu_angle_y, double imu_angle_z,
                    double imu_rate_x, double imu_rate_y, double imu_rate_z,
                    double imu_accel_x, double imu_accel_y,
                    double imu_accel_z) {
  geometry_msgs::msg::Quaternion Orientation_Quaternion;
  geometry_msgs::msg::Vector3 AngularVelocity3D;
  geometry_msgs::msg::Vector3 LinearAccel3D;

  Quaternion ConvertedQuaternion = ToQuaternion(
      imu_angle_x * DEG_to_RAD, imu_angle_y * DEG_to_RAD,
      imu_angle_z * DEG_to_RAD); // Covert angles to quaternion, uses rad/s
  Orientation_Quaternion.x = ConvertedQuaternion.x;
  Orientation_Quaternion.y = ConvertedQuaternion.y;
  Orientation_Quaternion.z = ConvertedQuaternion.z;
  Orientation_Quaternion.w = ConvertedQuaternion.w;

  AngularVelocity3D.x = imu_rate_x * DEG_to_RAD;
  AngularVelocity3D.y = imu_rate_y * DEG_to_RAD;
  AngularVelocity3D.z = imu_rate_z * DEG_to_RAD;

  LinearAccel3D.x = imu_accel_x;
  LinearAccel3D.y = imu_accel_y;
  LinearAccel3D.z = imu_accel_z;

  if (imu_variance_known_b) {

    current_imu.orientation = Orientation_Quaternion;
    current_imu.orientation_covariance = {Orientation_X_Variance, 0.0, 0.0, 0.0,
                                          Orientation_Y_Variance, 0.0, 0.0, 0.0,
                                          Orientation_Z_Variance};

    current_imu.angular_velocity = AngularVelocity3D;
    current_imu.angular_velocity_covariance = {Rate_X_Variance, 0.0, 0.0, 0.0,
                                               Rate_Y_Variance, 0.0, 0.0, 0.0,
                                               Rate_Z_Variance};

    current_imu.linear_acceleration = LinearAccel3D;
    current_imu.linear_acceleration_covariance = {
        Accel_X_Variance, 0.0, 0.0, 0.0, Accel_Y_Variance, 0.0, 0.0, 0.0,
        Accel_X_Variance};
  } else {

    current_imu.orientation = Orientation_Quaternion;
    current_imu.orientation_covariance = {Variance_Unkown, 0.0, 0.0, 0.0,
                                          Variance_Unkown, 0.0, 0.0, 0.0,
                                          Variance_Unkown};

    current_imu.angular_velocity = AngularVelocity3D;
    current_imu.angular_velocity_covariance = {Variance_Unkown, 0.0, 0.0, 0.0,
                                               Variance_Unkown, 0.0, 0.0, 0.0,
                                               Variance_Unkown};

    current_imu.linear_acceleration = LinearAccel3D;
    current_imu.linear_acceleration_covariance = {
        Variance_Unkown, 0.0, 0.0, 0.0, Variance_Unkown, 0.0, 0.0, 0.0,
        Variance_Unkown};
  }
}

} // namespace sd
