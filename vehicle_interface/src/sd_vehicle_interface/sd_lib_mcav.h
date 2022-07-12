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


#include <can_msgs/msg/frame.hpp>

/*
This file contains the functions originally declared in sd_lib_h.h, but actually includes
implementations that were created based on information found in the StreetDrone User Manual V1.6,
and through investigating the CAN frames sent by the original pre-compiled shared library libsd_interface_lib.so.
This file effectively replaces that library, so that we can re-compile these functions so that they can 
be run on an ARM-based computer (the Nvidia Drive PX2). The library as supplied was compiled
only for x86-64 architecture.
*/

// Big table of values to speed up CRC calculations
// Generated based on the parameters specified by StreetDrone:
// Polynomial: 0x1D
// Initial CRC value: 0xFF
static unsigned char const crc8_data[] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
	0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
	0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
	0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
	0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
	0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
	0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
	0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
	0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
	0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
	0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
	0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
	0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
	0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
	0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
	0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

namespace sd{
	//**************************************************
	/*				SD RX FUNCTIONS				*/
	//**************************************************
	void SetCRC(can_msgs::msg::Frame& frame, uint8_t aliveCount) {
		// See the CRC calculation in the StreetDrone user manual
		// The StreetDrone's ECU checks that this checksum matches with
		// what is expected, in order to verify integrity of the frame
		// that it receives.

		// CRC Parameters (these make more sense if you read up on CRC)
		// Polynomial: 0x1D
		// Initial CRC value: 0xFF

		// This function also updates the aliveCount
		frame.data[1] = aliveCount; // Customer_Control_1_Alive (byte 1)

		uint8_t crc = 0xFF;
		crc = crc8_data[crc ^ frame.data[1]];
		crc = crc8_data[crc ^ frame.data[2]];
		crc = crc8_data[crc ^ frame.data[3]];
		crc = crc8_data[crc ^ frame.data[4]];
		crc = crc8_data[crc ^ frame.data[5]];
		crc = crc8_data[crc ^ frame.data[6]];
		crc = crc8_data[crc ^ frame.data[7]];
		frame.data[0] = crc;
	}
	
	/*ParseRxCanDataSDCan
	This function parses an input can frame, checks it's ID and updates appropriate variables with freshest data*/
    void ParseRxCANDataSDCan(can_msgs::msg::Frame& frame, double& CurrentLinearVelocity_Mps, bool& AutomationGranted_B, bool& AutomationArmed_B) {
		// Check which kind of frame we have received and update data accordingly
		if (frame.id == 0x100) { // StreetDrone_Control_1
			bool steer_automation_available = frame.data[7] & 0x80; // bit 56
			bool steer_automation_granted = frame.data[7] & 0x40; // bit 57
			bool torque_automation_available = frame.data[7] & 0x8l; // bit 60
			bool torque_automation_granted = frame.data[7] & 0x4; // bit 61
			AutomationArmed_B = steer_automation_available && torque_automation_available; // I am only assuming this is the calculation, could be good to double check
			AutomationGranted_B = steer_automation_granted && torque_automation_granted;
		} else if (frame.id == 0x102) { // StreetDrone_Data_1
			uint8_t speed_actual = frame.data[0]; // (byte 1) speed in km/h
			CurrentLinearVelocity_Mps = ((double) speed_actual) / 3.6; // convert to m/s
		}
	}
	/*Inputs
	can_msgs::msg::Frame& ReceivedFrameCAN : Raw socketcan frame in format of can_msgs/Frame.h, this is the RX can stream
	double& CurrentLinearVelocity_Mps The function shall update this variable with the latest Linear Velocity as read from the CAN bus in Mps
	bool& AutomationGranted_B,  This function shall set this variable to TRUE if the latest received can data condirms vehicle is in Automated Mode
	bool& AutomationArmed_B  This function shall set this variable to TRIE if the latest received CAN data confirms that the vehicle is armed for autonomous mode*/
	
	//**************************************************
	/*				SD TX FUNCTIONS				*/
	//**************************************************
	
	/*
	InitSDInterfaceControl & InitSDInterfaceFeedback
	These functions  assign the appriiate CAN IDs for the Control and Feedback frames of the StreetDrone CAN protocol
	They should be run on a socketcan frame in format of can_msgs/Frame.h
	Perform these function on initialisation before main loop
	The Feedback message is optional*/
    void InitSDInterfaceControl(can_msgs::msg::Frame& frame) {
		frame.dlc = 8; // each frame has 8 bytes of data
		frame.id = 0x101; // customer Control frame id constant
	}
	void InitSDInterfaceFeedback(can_msgs::msg::Frame& frame) {
		frame.dlc = 8; // each frame has 8 bytes of data
		frame.id = 0x103; // customer Feedback frame id
	}
	/*Inputs
	can_msgs::msg::Frame& CustomerControlCANTx/CustomerFeedbackCANTx to be initialised*/
	
	//Request Autonomous Control of the Vehicle
    void RequestAutonomousControl(can_msgs::msg::Frame& frame,  uint8_t aliveCount) {
		frame.dlc = 8; // length of data in bytes
		frame.id = 0x101; // CustomerControl frame id: 0x101
		frame.data[0] = 0; // Customer_Control_1_CRC (byte 0) (8-bit unsigned integer, calculation in SetCRC)
		frame.data[1] = 0; // Customer_Control_1_Alive (byte 1) (8-bit unsigned integer, increments by 1 every message)
		frame.data[2] = 0; // Steer_Request (byte 2) (8-bit signed integer, units %, min value -100 decimal == 0x9C, max value 100 decimal == 0x64)
		frame.data[3] = 0; // Torque_Request (byte 3) (as for Steer_Request)
		frame.data[4] = 0; // Reserved (byte 4)
		frame.data[5] = 0; // Reserved (byte 5)
		frame.data[6] = 0; // Reserved (byte 6)
		frame.data[7] = 0x11; // sets SteerAutomationRequest and TorqueAutomationRequest to 1.
		// (byte 7 contains two bit flags that set SteerAutomationRequest and TorqueAutomationRequest to true or false)
		// 0x11 in binary is 0b10001, so it sets bit 56 and bit 60 from the start of the CAN frame

		sd::SetCRC(frame, aliveCount); // 
	}
	/*inputs
	can_msgs::msg::Frame& CustomerControlCANTx :The SD Interface Control Message after initialisation
	uint8_t AliveCounter_Z : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/

	//ResetControlCanData
	//This function resets all but the alive counter to 0
	//This will handback control to the safety driver
    void ResetControlCanData(can_msgs::msg::Frame& frame, uint8_t aliveCount) {
		frame.data[0] = 0;
		frame.data[1] = 0;
		frame.data[2] = 0;
		frame.data[3] = 0;
		frame.data[4] = 0;
		frame.data[5] = 0;
		frame.data[6] = 0;
		frame.data[7] = 0;

		// Set CRC value correctly
		sd::SetCRC(frame, aliveCount);
	}

	// This function just updates the ControlAlive field
	void UpdateControlAlive(can_msgs::msg::Frame& frame, uint8_t aliveCount) {
		SetCRC(frame, aliveCount); // actual functionality is performed by SetCRC
	}

	/*Inputs
	can_msgs::msg::Frame& CustomerControlCANTx :The SD Interface Control Message after initialisation
	uint8_t AliveCounter_Z : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/

	//PopControlCANData
	//Populates the Control Tx message to the vehicle
    void PopControlCANData(can_msgs::msg::Frame& frame, int8_t torque_request_pc, int8_t steer_request_pc, uint8_t aliveCount) {
		frame.data[2] = steer_request_pc; // Steer_Request (byte 2) (8-bit signed integer, units %, min value -100 decimal == 0x9C, max value 100 decimal == 0x64)
		frame.data[3] = torque_request_pc; // Torque_Request (byte 3) (as for Steer_Request)
		SetCRC(frame, aliveCount);
	}
	/*Inputs
	can_msgs::msg::Frame& CustomerControlCANTx  The SD Interface Control Message after initialisation
	int8_t FinalDBWTorqueRequest_Pc:The Torque percentage requested of the vehicle
	int8_t FinalDBWSteerRequest_Pc: The Steer Percentage requested of the vehicle
	uint8_t AliveCounter_Z  : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/
	
	//Populates the Feedback CAN message (Optional)
    void PopFeedbackCANData(can_msgs::msg::Frame&, int, int, int, int, double, double);
	// This hasn't been re-implemented since it is not required for customers in general operation
	// and there wasn't much info available.

	/*Inputs
	void can_msgs::msg::Frame& ControllerFeedbackCANTx
	int P_Contribution_Pc :The Contribution to final torque by the P term
	int I_Contribution_Pc :The Contribution to final torque by the I term
	int D_Contribution_Pc :The Contribution to final torque by the D term
	int FF_Contribution_Pc :The Contribution to final torque by Feedorward control
	double TargetLinearVelocity_Mps :The Target speed (feedback only)
	double TargetAngularVelocity_Degps: The Target Angular velocity (Feedback Only)*/

}

