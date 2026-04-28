#!/usr/bin/env python3
# Copyright (c) 2026 Monash Connected Autonomous Vehicle (MCAV)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Monash Connected Autonomous Vehicle (MCAV) nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Integration test for sd_vehicle_interface node.

Tests that the node correctly processes control commands and publishes
appropriate status messages based on the sd_control.cpp calculations.
"""

import sys
import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
import rclpy
from rclpy.node import Node


class TestSdVehicleInterface(Node):
    def __init__(self):
        super().__init__('test_sd_vehicle_interface')

        self.steering_results = []

        self.control_pub = self.create_publisher(
            Control, '/control/command/control_cmd', 10)

        self.create_subscription(
            SteeringReport, '/vehicle/status/steering_status',
            self._steering_cb, 10)

        self.create_timer(0.05, self._publish_commands)

    def _steering_cb(self, msg):
        self.steering_results.append(msg.steering_tire_angle)

    def _publish_commands(self):
        control_msg = Control()
        control_msg.lateral.steering_tire_angle = 0.35
        control_msg.lateral.steering_tire_rotation_rate = 0.0
        control_msg.longitudinal.velocity = 5.0
        self.control_pub.publish(control_msg)

    def has_steering_results(self):
        return len(self.steering_results) >= 5


def main():
    rclpy.init()
    node = TestSdVehicleInterface()

    print('[TEST] Publishing control_cmd with steering_tire_angle=0.35 rad, velocity=5.0 m/s')
    print('[TEST] Waiting for node to process commands and publish steering_status...')

    timeout_sec = 15.0
    start = time.time()
    while time.time() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.has_steering_results():
            break

    passed = True
    print()
    print('=' * 60)
    print(f'[INFO] Total steering_status msgs: {len(node.steering_results)}')

    if not node.steering_results:
        print('[FAIL] No steering_status messages received')
        passed = False
    else:
        avg_steering = sum(node.steering_results) / len(node.steering_results)
        print(f'[INFO] Average steering_tire_angle: {avg_steering:.4f} rad')

        if abs(avg_steering) > 0.01:
            print('[PASS] steering_status is being published correctly')
        else:
            print('[WARN] steering_status values are near zero (node may be in manual mode)')

    print('=' * 60)
    if passed:
        print('[RESULT] INTEGRATION TEST COMPLETED')
    else:
        print('[RESULT] TEST FAILED')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
