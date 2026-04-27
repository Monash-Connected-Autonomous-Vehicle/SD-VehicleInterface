// Copyright (c) 2020 StreetDrone Limited
//
//
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
//    * Neither the name of the StreetDrone Limited nor the names of its
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

// Author: Fionán O'Sullivan
// Based on original work of: Efimia Panagiotaki

#ifndef SD_VEHICLE_INTERFACE__SD_TYPEDEFS_H_
#define SD_VEHICLE_INTERFACE__SD_TYPEDEFS_H_

#include "rclcpp/rclcpp.hpp"
#define KPH_to_MPS (0.278)


typedef union CAN_frame_t {
  uint8_t byte[8];
  uint16_t word[4];
  uint32_t dword[2];
  uint64_t frame;
} CAN_frame_t;

// Used to pack CAN words (16 bit signals).
typedef union CAN_word_t {
  uint8_t bytes[2];
  uint16_t word;
} CAN_word_t;

typedef union float_bits_converter {
  unsigned int integer_can;
  float float_can;
} float_bits_converter;

#endif  // SD_VEHICLE_INTERFACE__SD_TYPEDEFS_H_
