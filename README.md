# Particle Filter Project
Self-Driving Car Engineer Nanodegree Program

This project implements a PID (Proportional, Integral, Derivative) controller in C++ for nagivating a car around a test track in a simulator. The speed, angle, and cross-track error (CTE) is read out for each time step in the simulator, and used as input for the PID controller.

## PID controller
The PID controller is a simple feedback algorithm that calculates the desired steering angle of the car based on its CTE, where CTE is the difference between the desired state (center of lane) vs the measured state (position of car). The formula used for determining steering angle is

*steer_angle = Kp * p_error - Kd * d_error - Ki * i_error*



Below is a screenshot of the final run output

![PID output](./final_output.png)

Below is a video of the final run

![PID video run](./final_run.mkv)


