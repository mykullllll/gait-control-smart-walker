# Feedback Validation Documentation

## Objective 
The purpose of this validation is to find the optimal gains for a PID control system used as a corrective velocity if the user leaves the desired "zone/distance" to the walker. The metrics used to assess each gain combination is determined under the criteria of smoothness after unpredictable posotion disturbance, given sampling rate, latency, deadband, and motor limits. 

The metrics used are shown below:

- Integral Absolute Error (IAE): Cumulative error per gain combination over time $$IAE = \sum_k |e_k| \Delta t$$
Where:
$e_k$: The error value at time step $k$.

- Root Mean Square Error (RMSE): Penalizes occasional large position errors more strongly: $$RMSE = \sqrt{\frac{1}{N}\sum_k e_k^2}$$
Where: $e_k$: The error value at time step $k$.

- Maximum Error

- Root Mean Square (RMS): Calculates average magnitude of Acceleration and Jerk to measure comfort throughout trial: $$RMS = \sqrt{\frac{1}{N}\sum_k a_k^2}$$

- Overshoot: Position crosses to the opposite side of the desired position

In order to easily debug and make sure all parts of the program are working correctly, I've added features incrementally shown as "Version 1,2,3 etc..."  
Note:(All Versions have the previous implementations included)


## Test Configuaration Parameters

Position error:

    x_pelvis = (x_left + x_right)/2
    e = x_pelvis - x_desire


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
| Simulation duration | 16.67 s |

# Version 1 Ideal Proportional Controller

## Objective
Determine how proportional gain will affect metrics specified above. 
This test is meant to establish a reference range of gains. It is not a final validation of the physical walker controller.

## Gains tested

    0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0

## Results

See the [position gain baseline results](gait_baseline_data.md).

# Version 2 Patient Velocity + PID

## Objective

Determine how Proportional Integral and Derivative Gain will affect settling time, overshoot, and peak velocity with realistic change in positions. 
This test is meant to establish a reference range of gains based off postitional change for PID. 

## Procedure
A set of simulated forward and backward velocities of a patient is shown below. Every 2 seconds the next patient velocity is used changing the position of the user relative to the walker. 

Patient Velocity(m/s)       0,-0.2, 0.2, -0.16, 0.12,-0.20 


## Controller Equation:

$$
\begin{aligned}
\text{Velocity} = k_p \cdot e + k_i \int_0^t e \, dt + k_d \cdot \dot{e}
\end{aligned}
$$


## Gains tested

* k_p_values = [0,0.25,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0]
* k_i_values = [0.0, 0.05, 0.10, 0.20,0.3,0.4,0.5]
* k_d_values = [0.0, 0.02, 0.05, 0.10,0.15,0.20,0.25]


## Results

See the [Dynamic Gain Data Results](dynamic_gain_data.md).

Gains around K_p = 2-3 with little to zero integral action form the most promising area. Derivative gains provides only small tracking benefit, and it's value cannot be decided until latency and sensor noise are included. There is no "optimal" gain since it depends on what you're optimizing for but we can narrow down our choices based off of our RMSE values. The minimum RMSE positional value is at k_p = 3, k_i = 0, k_d = 0.25 with RMSE (m) = 0.065315953 and RMS jerk: 1.165114 m/s³ . For a less aggressive candidate, k_p=2, k_i=0, k_d=0 with RMSE (m) = 0.065315953 \

This less aggressive candidate produces:

* 12.75% higher position RMSE
* 21.82% lower RMS acceleration
* 20.25% lower RMS jerk
* 20.33% lower maximum error
* About 20.75% higher IAE


Therefore there is no single optimal gain, rather a tradeoff between smoothness vs tracking. In addition derivaitve gain barely produced any improvement while integral gains didn't actually help improve steady state error due due to the constant change in velocity. These values will be evaluated again under latency and noise shown below.\

# Version 3 Real time Motor and Hardware Latency + Positional Noise

## Objective
Determine how motor and hardware latency will affect metrics above. The latency added will simulate motor response commands due to ramping as well as communication delay (dead time) between the flowchart shown below. Noise is also added representative of positional error from LiDAR scans. 


# Version 4 Feedforward Velocity Correction

## Objective 
Determine optimal gains from region given feedforward velocities from Adaptive Frequency Oscillators implemented during deadband zones. The PID should correct the positional errors when the user walks outside of the deadband zone. 




