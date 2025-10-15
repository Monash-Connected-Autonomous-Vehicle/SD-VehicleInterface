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

#include <can_msgs/msg/frame.h>
#include <cstdint>

namespace sd {
//**************************************************
/*				SD RX FUNCTIONS				*/
//**************************************************

/*
 * parse CAN frame, check ID and update appropriate variables with new data
 *
 * Inputs:
 * - can_msgs::msg::Frame& ReceivedFrameCAN: Raw socketcan frame in format of
 *   can_msgs/Frame.h, this is the RX can stream
 * - double& CurrentLinearVelocity_Mps: updated with the latest Linear Velocity
 *   as read from the CAN bus in Mps
 * - bool& automation_granted_b: set to TRUE if the CAN data confirms vehicle is
 *   in Automated Mode
 * - bool& automation_armed_b: set to TRIE if the CAN data confirms vehicle is
 *   armed for autonomous mode
 */
void ParseRxCANDataSDCan(can_msgs::msg::Frame &, double &, int8_t &, bool &, bool &);

//**************************************************
/*				SD TX FUNCTIONS				*/
//**************************************************

/*
 * Assigns CAN IDs for the Control+Feedback frame of the StreetDrone CAN
 * protocol. Should be run on a socketcan frame in can_msgs/Frame.h format.
 * Run on initialisation before main loop. Feedback message is optional
 *
 * Inputs:
 * - can_msgs::msg::Frame& customer_control_can_tx/CustomerFeedbackCANTx to be
 *   initialised
 */
void InitSDInterfaceControl(can_msgs::msg::Frame &);
void InitSDInterfaceFeedback(can_msgs::msg::Frame &);

/*
 * Request autonomous control of the vehicle
 * Inputs:
 * - can_msgs::msg::Frame& customer_control_can_tx :The SD Interface Control
 *   Message after initialisation
 * - uint8_t alive_counter_z : An Alive counter. Increment this variable by 1
 *   each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*
 */
void RequestAutonomousControl(can_msgs::msg::Frame &, uint8_t);

/*
 * Resets all but the alive counter to 0. Control returned to safety driver.
 * Inputs
 * - can_msgs::msg::Frame& customer_control_can_tx: The SD Interface Control
 *   Message after initialisation
 * - uint8_t alive_counter_z: Alive counter, increment by 1 each loop (min
 * 200Hz). Protects again stale CAN data
 */
void ResetControlCanData(can_msgs::msg::Frame &, uint8_t);
void UpdateControlAlive(can_msgs::msg::Frame &, uint8_t);

/*
 * Set steer/torque request in Customer_Control_1 CAN frame
 *
 * Inputs:
 * - can_msgs::msg::Frame& customer_control_can_tx:  The SD Interface Control
 *   Message after initialisation
 * - int8_t final_dbw_torque_request_pc: The Torque percentage requested of the
 *   vehicle
 * - int8_t final_dbw_steer_request_pc: The Steer Percentage requested of the
 *   vehicle
 * - uint8_t alive_counter_z: Alive counter, increment by 1 each loop (min
 *   200Hz). Protects again stale CAN data*
 */
void PopControlCANData(can_msgs::msg::Frame &, int8_t, int8_t, uint8_t);

/**
 * Set auxiliary controls in Customer_Control_2 CAN Tx frame
 */
void PopControl2CANData(can_msgs::msg::Frame &frame, bool hazardLightsRequest,
                        bool leftIndicatorRequest, bool rightIndicatorRequest,
                        uint8_t aliveCount);

/*
 * Populates the Feedback CAN message (optional, unimplemented)
 *
 * Inputs:
 * - void can_msgs::msg::Frame& ControllerFeedbackCANTx
 * - int p_contribution_pc: contribution to final torque by P term
 * - int i_contribution_pc: contribution to final torque by I term
 * - int d_contribution_pc: contribution to final torque by D term
 * - int ff_contribution_pc: contribution to final torque by Feedforward control
 * - double TargetLinearVelocity_Mps: target speed (feedback only)
 * - double TargetAngularVelocity_Degps: target angular velocity (feedback only)
 */
void PopFeedbackCANData(can_msgs::msg::Frame &, int, int, int, int, double,
                        double);

} // namespace sd
