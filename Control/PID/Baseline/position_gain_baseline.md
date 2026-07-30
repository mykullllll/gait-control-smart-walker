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

## Gains tested

    0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0

## Metric Definitions

- Settling time: First time the position enters deadband and remains there
- Final error: Final pelvis position minus desired position.
- Overshoot: Position crosses to the opposite side of the desired position


## Results

See the [position gain baseline results](gait_baseline_data.md).