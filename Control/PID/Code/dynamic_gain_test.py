import sys
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import csv
from pathlib import Path
from itertools import product


control_directory = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(control_directory))

from AFO_PID import main_loop


current_time_history=[]
average_position_history=[]
position_error_history=[]
commanded_velocity_linear_history =[]



k_p_values = [0,0.25,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0]
k_i_values = [0.0, 0.05, 0.10, 0.20]
k_d_values = [0.0, 0.02, 0.05, 0.10]
gain_results=[]



for k_p,k_i,k_d in product(k_p_values,k_i_values,k_d_values):
    controller = main_loop(
        fs=6,
        wheel_radius=0.1143,
    )

    # Keep freeze detection out of the position-controller test.
    controller.ramp_complete_time = None
    controller.freeze_detection_armed = False

    #Gains
    controller.walker.k_p = k_p
    controller.walker.k_i = k_i
    controller.walker.k_d = k_d

    #Metrics
    settle = False
    overshoot_status = False
    prev_state=False
    deadband_samples = 0
    overshoot=0
    counter=0
    oscillation = 0
    integral_abs_error=0
    peak_velocity = 0.0


    # Skip the real 15-second calibration for this test.
    controller.calibrated = True
    controller.x_d = -0.40
    controller.velocity_gain = 0
    controller.cal_stride = 0.30
    controller.previous_stride = 0.30
    controller.raw_frequency = 0.60
    controller.cadence = 0.60
    controller.prev_cadence = 0.60



    #Simulated Velocity Changes Variables
    disturbance_velocities = [0,-0.2, 0.2, -0.16, 0.12, -0.20]
    disturbance_index = 0
    disturbance_interval_s = 2.0
    movement_duration_s=0.5

    disturbance_interval_steps = round(
        disturbance_interval_s * controller.fs
    )

    movement_duration_steps = round(
        movement_duration_s * controller.fs
    )

    initial_position = -0.50
    simulation_steps= 100
    dt = 1.0 / controller.fs
    average_position = initial_position
    disturbance_index = 0



    # Begin directly in fixed-gait tracking mode.
    controller.assist_ramping = False
    controller.afo_enabled = False


    for index in range(simulation_steps):

        current_time = index * dt
        step_in_interval = index % disturbance_interval_steps   

        if step_in_interval < movement_duration_steps:
            patient_velocity = disturbance_velocities[disturbance_index]
        else:
            patient_velocity = 0 
        

        # Equal positions are sufficient for a position-only test.
        left_x = average_position
        right_x = average_position

        result = controller.step_from_legs(
            current_time=current_time,
            encoder_velocity=0.0,
            left_x=left_x,
            right_x=right_x,
            isoccluded=False,
        )

        if result is None or result[0] is None:
            continue
        position_error = (controller.walker.error_history[-1])
        commanded_velocity_linear = result[0] * 0.1143
        average_position+= (patient_velocity * dt) - (commanded_velocity_linear * dt)

        peak_velocity = max(peak_velocity,abs(commanded_velocity_linear))
        simulation_error = (average_position - controller.x_d)

        integral_abs_error += (abs(simulation_error) * dt)        
        
        # print(
        #     f"time={current_time:.5f}, "
        #     f"disturbance={disturbance_index}, "
        #     f"patient_velocity={patient_velocity:.5f}, "
        #     f"position={average_position:.5f}, "
        #     f"command={commanded_velocity_linear:.5f} m/s"
        # )

        # Change velocity after completing the 2-second interval.
        if (index + 1) % disturbance_interval_steps == 0:
            disturbance_index += 1
            disturbance_index %= len(disturbance_velocities)



        current_time_history.append(current_time)
        average_position_history.append(average_position)
        position_error_history.append(position_error)
        commanded_velocity_linear_history.append(commanded_velocity_linear)

        if abs(controller.x_d - average_position) <= controller.walker.position_deadband:
            prev_state=True
            counter +=1
            deadband_samples += 1
            if counter > 30:
                settle = True
                settling_time = current_time + dt - (counter - 1) * dt
                
        else:
            counter=0
            if prev_state is True:
                oscillation+=1
            prev_state = False

        if average_position > controller.x_d :
            overshoot_status=True





    gain_results.append(    {

        "k_p": controller.walker.k_p,
        "k_i": controller.walker.k_i,
        "k_d": controller.walker.k_d,
        "Error Integral": {integral_abs_error},
        "Percentage of Time in Deadband (%)" : round(100*deadband_samples/(simulation_steps),2),
        "Max velocity m/s" :{peak_velocity},
        "Overshoot": {overshoot_status},
        "Number of oscillations around Deadband" : {oscillation}
    }
    )

    print(
        f"Kp={controller.walker.k_p}, "
        f"Ki={controller.walker.k_i}, "
        f"Kd={controller.walker.k_d}, "
        f"Error Integral {integral_abs_error}"
        f"Percentage of Time in Deadband (%) = {deadband_samples/(simulation_steps*dt)}%, "
        f"Max velocity = {peak_velocity} m/s "
        f"Overshoot={overshoot_status} "
        f"Number of deadband exits {oscillation}"
    )












csv_path = (
    Path(__file__).resolve().parents[1]
    / "Data"
    / "dynamic_gain_results.csv"
)

fieldnames = [
    "k_p",
    "k_i",
    "k_d",
    "Error Integral",
    "Percentage of Time in Deadband (%)",
    "Max velocity m/s",
    "Overshoot",
    "Number of oscillations around Deadband",
]

with csv_path.open(
    "w",
    newline="",
    encoding="utf-8",
) as csv_file:
    writer = csv.DictWriter(
        csv_file,
        fieldnames=fieldnames,
    )
    writer.writeheader()
    writer.writerows(gain_results)

print(f"Saved {len(gain_results)} results to {csv_path}")

#Saving to gait_baseline_data.md file
markdown_path = Path(__file__).with_name("dynamic_gain_data.md")

with markdown_path.open("w") as md:
    print("# Gain Results\n", file=md)
    print(
        "| k_p| k_i | k_d | Error Integral | Percentage of Time in Deadband (%) | Max velocity m/s | Overshoot | Number of oscillations around Deadband |",
        file=md,
    )
    print("|---:|---:|---:|---:|:---:|:---:|:---:|:---:|", file=md)

    for result in gain_results:
        print(
            f"| {result['k_p']} "
            f"| {result['k_i']} "
            f"| {result['k_d']} "
            f"| {result['Error Integral']} "
            f"| {result['Percentage of Time in Deadband (%)']}"
            f"| {result['Max velocity m/s']}"
            f"| {result['Overshoot']}"
            f"| {result['Number of oscillations around Deadband']} |",
            file=md,
        )

print(f"Saved to {markdown_path}")