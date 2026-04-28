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

#include <cmath>

#include <gtest/gtest.h>  // NOLINT(build/include_order)

#include "sd_vehicle_interface/sd_gps_imu.h"

TEST(SDGpsImuTest, ToQuaternionProducesNormalizedOutput)
{
  const sd::Quaternion quaternion = sd::ToQuaternion(M_PI / 4.0, M_PI / 6.0, 0.0);
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);

  EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(SDGpsImuTest, ParseRxCANDataOXTSCanParsesYawRateFrame)
{
  can_msgs::msg::Frame frame;
  frame.id = 1544;
  frame.dlc = 8;
  frame.data = {100, 0, 200, 0, 44, 1, 0, 0};

  double current_velocity = 0.0;
  double longitude = 0.0;
  double latitude = 0.0;
  double angle_x = 0.0;
  double angle_y = 0.0;
  double angle_z = 0.0;
  double rate_x = 0.0;
  double rate_y = 0.0;
  double rate_z = 0.0;
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;

  sd::ParseRxCANDataOXTSCan(
    frame, current_velocity, longitude, latitude,
    angle_x, angle_y, angle_z,
    rate_x, rate_y, rate_z,
    accel_x, accel_y, accel_z);

  EXPECT_DOUBLE_EQ(1.0, rate_x);
  EXPECT_DOUBLE_EQ(2.0, rate_y);
  EXPECT_DOUBLE_EQ(3.0, rate_z);
}

TEST(SDGpsImuTest, PackImuMessageProducesConvertedValues)
{
  sensor_msgs::msg::Imu imu_message;

  sd::PackImuMessage(
    true, imu_message,
    10.0, 20.0, 30.0,
    1.0, 2.0, 3.0,
    0.5, -0.1, 9.8);

  EXPECT_NEAR(imu_message.angular_velocity.x, 1.0 * DEG_to_RAD, 1e-6);
  EXPECT_NEAR(imu_message.angular_velocity.y, 2.0 * DEG_to_RAD, 1e-6);
  EXPECT_NEAR(imu_message.angular_velocity.z, 3.0 * DEG_to_RAD, 1e-6);
  EXPECT_NEAR(imu_message.linear_acceleration.x, 0.5, 1e-6);
  EXPECT_NEAR(imu_message.linear_acceleration.y, -0.1, 1e-6);
  EXPECT_NEAR(imu_message.linear_acceleration.z, 9.8, 1e-6);
  EXPECT_TRUE(std::isfinite(imu_message.orientation.w));
}
