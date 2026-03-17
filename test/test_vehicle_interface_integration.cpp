#include <gtest/gtest.h>

#include "sd_lib_mcav.h"

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

