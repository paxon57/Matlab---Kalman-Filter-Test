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

![image](https://github.com/user-attachments/assets/f1691e87-a495-4122-82d3-d087aa359e44)

Position error: Noise of up to 30 meters, Average of 18 meters

Velocity error: Noise of up to 25 m/s, Average of 6 m/s

Heading error: Noise of up to 15 degrees, Average of 0.4 degrees

![image](https://github.com/user-attachments/assets/c9c2896b-38c9-4b8a-98e3-088d402f1a31)


Known Bug: Sometimes glitches out when measurements cross south from the radar.
