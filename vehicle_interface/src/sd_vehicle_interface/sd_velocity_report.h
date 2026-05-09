#pragma once

void ComputeLongitudinalVelocity(double& longitudinal_velocity, double IMU_Accel_X)
{
    if(longitudinal_velocity < 0)
    {
        longitudinal_velocity = 0;
    }

    longitudinal_velocity += IMU_Accel_X * 0.05; // dt = 5ms = 0.05s as stated in the main loop rate
}

void ComputeLateralVelocity()
{
    
}