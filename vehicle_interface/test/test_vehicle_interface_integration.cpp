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

#include "sd_vehicle_interface/sd_lib_mcav.h"

TEST(SDVehicleInterfaceProtocolTest, InitControlFrameSetsExpectedMetadata)
{
  can_msgs::msg::Frame frame;

  sd::InitSDInterfaceControl(frame);

  EXPECT_EQ(8, frame.dlc);
  EXPECT_EQ(0x101u, frame.id);
}

TEST(SDVehicleInterfaceProtocolTest, RequestAutonomousControlSetsAliveCounterAndRequestBits)
{
  can_msgs::msg::Frame frame;

  sd::RequestAutonomousControl(frame, 7);

  EXPECT_EQ(8, frame.dlc);
  EXPECT_EQ(0x101u, frame.id);
  EXPECT_EQ(7, frame.data[1]);
  EXPECT_EQ(0x11, frame.data[7]);
}

TEST(SDVehicleInterfaceProtocolTest, PopControlCANDataWritesSteerAndTorqueBytes)
{
  can_msgs::msg::Frame frame;
  sd::InitSDInterfaceControl(frame);

  sd::PopControlCANData(frame, 12, -34, 3);

  EXPECT_EQ(static_cast<uint8_t>(-34), frame.data[2]);
  EXPECT_EQ(static_cast<uint8_t>(12), frame.data[3]);
  EXPECT_EQ(3, frame.data[1]);
}

TEST(SDVehicleInterfaceProtocolTest, ParseRxCANDataSDCanReadsLowResolutionSpeed)
{
  can_msgs::msg::Frame frame;
  frame.id = 0x102;
  frame.dlc = 8;
  frame.data = {100, 0, 0, 0, 0, 0, 0, 0};

  double current_velocity = 0.0;
  bool automation_armed = false;
  bool automation_granted = false;

  sd::ParseRxCANDataSDCan(frame, current_velocity, automation_armed, automation_granted);

  EXPECT_NEAR(0.277778, current_velocity, 1e-6);
  EXPECT_FALSE(automation_armed);
  EXPECT_FALSE(automation_granted);
}
