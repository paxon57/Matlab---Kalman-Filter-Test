# Matlab - Kalman Filter Test

This is just a personal learning project

Objective: Based on Radar reading estimate plane position

Map Size: 10 km x 10 km (Adjustable)
Random Radar Positioning
Random Flight Pattern

## Kalman Filter
Check "Kalman Filter" branch for working Kalman Filter

In this scenario radar provides position information with 150 meter noise.

Tweak settings at the top of main.m

Run main.m to perform the simulation

## Extended Kalman Filter
Main branch contains EKF

Tweak and run main.m

In this scenario radar provides only distance and Direction.
Distance noise is +/- 20 meters.
Direction noise is +/- 0.5 degrees.
Velocity and direction is not measured, only estimated.
The plane (tracking target) is randomly turning and accelerating.

![image](https://github.com/user-attachments/assets/8de3cab9-213a-4b4e-aae8-a738e43148ce)

Position error: Average of 10 meters

Velocity error: Average of 1.05 m/s

Heading error: Average of 0.004 degrees

![image](https://github.com/user-attachments/assets/b7eedc53-d4a7-4620-862b-7c02ac4ecc7e)

![image](https://github.com/user-attachments/assets/81b1af7e-c9e7-46f2-b5a0-b162521e7328)

Known Bug: Sometimes glitches out when measurements cross south from the radar.
