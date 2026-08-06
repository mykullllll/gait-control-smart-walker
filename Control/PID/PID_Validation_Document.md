# Feedback Validation Documentation

## Objective 
The purpose of this validation is to find the optimal gains for a PID control system used as a corrective velocity if the user leaves the desired "zone/distance" to the walker. In order to easily debug and make sure all parts of the program are working correctly, I've added features incrementally as shown below. 

Position error:

    x_pelvis = (x_left + x_right)/2
    e = x_pelvis - x_desire

## Test Configuaration Parameters

| Parameters |
|---|
| Sampling frequency | 6 Hz |
| Timestep | 0.16667 s |
| Desired position | -0.40 m |
| Initial position | -0.50 m |
| Initial error | -0.10 m |
| Position deadband | ±0.02 m |
| Feedforward | Disabled |
| Pelvis filtering | Disabled (`alpha=1`) |
| Motor latency | 0 s |
| Motor response | Instantaneous |
| Acceleration limiting | Enabled |
| Simulation duration | 10 s |

## Metric Definitions

- Error Integral: Cumulative error per gain combination
- Percentage of time within desired range
- Maximum Velocity m/s
- Overshoot: Position crosses to the opposite side of the desired position
- Number of Overshoots

# Version 1 Ideal Proportional Controller

## Objective
Determine how proportional gain will affect settling time, overshoot, and peak velocity before realistic dynamics. 
This test is meant to establish a reference range of gains. It is not a final validation of the physical walker controller.

## Gains tested

    0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0

## Results

See the [position gain baseline results](gait_baseline_data.md).



# Version 2 Patient Velocity + PID

## Objective

Determine how Proportional Integral and Derivative Gain will affect settling time, overshoot, and peak velocity with realistic change in positions. 
This test is meant to establish a reference range of gains based of postitional change for PID. 

## Procedure
A set of simulated forward and backward velocities of a patient is shown below. Every 2 seconds the next velocity is commanded changing the position of the user relative to the walker. 

Patient Velocity(m/s)       0,-0.2, 0.2, -0.16, 0.12,-0.20 


## Controller Equation:

$$
\begin{aligned}
\text{Velocity} = k_p \cdot e + k_i \int_0^t e \, dt + k_d \cdot \dot{e}
\end{aligned}
$$


## Gains tested

* k_p_values = [0,0.25,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0]
* k_i_values = [0.0, 0.05, 0.10, 0.20]
* k_d_values = [0.0, 0.02, 0.05, 0.10]


## Results

See the [Dynamic Gain Data Results](dynamic_gain_data.md).






