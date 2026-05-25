#pragma once

void ComputeLongitudinalVelocity(float& longitudinal_velocity, double wheel_speed_scaled_mps)
{
    // Use vehicle wheel speed from CAN (frame 0x102) instead of integrating IMU
    longitudinal_velocity = static_cast<float>(wheel_speed_scaled_mps * UNDO_STREETDRONE_SCALING_FACTOR);
}

void ComputeLateralVelocity()
{
    
}

void ComputeHeadingRate(float& heading_rate, double IMU_Rate_Z)
{
    // IMU_Rate_Z is the yaw angular rate about the vehicle's vertical (Z) axis, in deg/s.
    heading_rate = static_cast<float>(IMU_Rate_Z * DEG_to_RAD);
}