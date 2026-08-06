# Idealized Position P-Controller Gain Baseline

## Objective

Determine how proportional gain will affect settling time, overshoot, and peak velocity before realistic dynamics. 
This test is meant to establish a reference range of gains. It is not a final validation of the physical walker controller.

## Controller Equation

Position error:

    e = x_pelvis - x_desired

Corrective velocity:

    v = Kp * e

Stationary-user position update:

    x_next = x_current - v * dt

The proportional gain has units of 1/s.

## Test Configuaration

| Parameter | Value |
|---|---:|
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


## Controller Equation

Position error:

    e = x_pelvis - x_desired

Corrective velocity:

$$
\begin{aligned}
\Velocity = k_p * error + k_i * $$\iint_t error\, dt$$ + k_d * dot{e} \\
\end{aligned}

## Gains tested

k_p_values = [0,0.25,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0]
k_i_values = [0.0, 0.05, 0.10, 0.20]
k_d_values = [0.0, 0.02, 0.05, 0.10]


## Results

See the [Dynamic Gain Data Results](dynamic_gain_data.md).




