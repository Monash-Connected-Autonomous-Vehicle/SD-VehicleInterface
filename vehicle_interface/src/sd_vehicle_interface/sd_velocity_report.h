#pragma once

#define MAX_LOGITUDINAL_VELOCITY 9.0
#define MIN_LOGITUDINAL_VELOCITY 0.0

void ComputeLongitudinalVelocity(float& longitudinal_velocity, double IMU_Accel_X)
{
    
    longitudinal_velocity += IMU_Accel_X * 0.05; // dt = 50ms = 0.05s as stated in IMU rate
    
}

void ComputeLateralVelocity()
{
    
}