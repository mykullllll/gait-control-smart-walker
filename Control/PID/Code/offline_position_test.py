import sys
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import csv
from pathlib import Path


pid_directory = Path(__file__).resolve().parent
control_directory = pid_directory.parent

sys.path.insert(0, str(pid_directory))
sys.path.insert(0, str(control_directory))

from AFO_PID import main_loop,WalkerController


current_time_history=[]
average_position_history=[]
position_error_history=[]
commanded_velocity_linear_history =[]



gain_values = [
    0.25,
    0.5,
    1.0,
    1.5,
    2.0,
    2.5,
    3.0,
    4.0,
    5.0,
    6.0,
]

gain_results=[]

for gain in gain_values:

    controller = main_loop(
        fs=6,
        wheel_radius=0.1143,
    )
    controller.walker.k_p = gain
    controller.walker.k_i = 0
    controller.walker.k_d = 0


    # Keep freeze detection out of the position-controller test.
    controller.ramp_complete_time = None
    controller.freeze_detection_armed = False

    controller.walker.k_p = gain
    convergence = False
    overshoot_status = False
    convergence_time = 0
    overshoot=0
    counter=0


    # Skip the real 15-second calibration for this test.
    controller.calibrated = True
    controller.x_d = -0.40
    controller.velocity_gain = 0
    controller.cal_stride = 0.30
    controller.previous_stride = 0.30
    controller.raw_frequency = 0.60
    controller.cadence = 0.60
    controller.prev_cadence = 0.60

    #Pelvis Positions
    initial_position = -0.50
    simulation_steps= 60
    dt = 1.0 / controller.fs
    average_position = initial_position

    # Begin directly in fixed-gait tracking mode.
    controller.assist_ramping = False
    controller.afo_enabled = False


    for index in range(simulation_steps):
        current_time = index * dt

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

        if result is None:
            continue
        position_error = (controller.walker.error_history[-1])
        commanded_velocity_linear = result[0] * 0.1143
        average_position-= commanded_velocity_linear * dt

        current_time_history.append(current_time)
        average_position_history.append(average_position)
        position_error_history.append(position_error)
        commanded_velocity_linear_history.append(commanded_velocity_linear)

        if abs(controller.x_d - average_position) < 0.02:
            counter +=1
            if counter > 30:
                convergence = True
                convergence_time = current_time + dt - (counter - 1) * dt
        else:
            counter=0

        if average_position > controller.x_d :
            overshoot_status=True
            

        print(
            f"time={current_time:.5f}, "
            f"position={average_position:.5f}, "
            f"error={position_error:.5f}, "
            f"command={commanded_velocity_linear:.5f} m/s"
        )

    gain_results.append(    {
        "gain": gain,
        "settling_time_s": (
            f"{convergence_time:.5f}"
            if convergence is True
            else "failed"
        ),
        "final_position_m": f"{average_position:.5f}",
        "final_error_m": f"{average_position - controller.x_d:.5f}",
        "overshoot": overshoot_status,
    }
)

csv_path = Path(__file__).with_name(
    "baseline_gain_data.csv"
)
fieldnames = [
    "gain",
    "settling_time_s",
    "final_position_m",
    "final_error_m",
    "overshoot",
]

with csv_path.open("w", newline="") as csv_file:
    writer = csv.DictWriter(
        csv_file,
        fieldnames=fieldnames,
    )
    writer.writeheader()
    writer.writerows(gain_results)

print(f"Results saved to {csv_path}")



#Saving to gait_baseline_data.md file
markdown_path = Path(__file__).with_name("baseline_gain_data.md")

with markdown_path.open("w") as md:
    print("# Gain Results\n", file=md)
    print(
        "| Gain | Settling Time | Final Position | Final Error | Overshoot |",
        file=md,
    )
    print("|---:|---:|---:|---:|:---:|", file=md)

    for result in gain_results:
        print(
            f"| {result['gain']} "
            f"| {result['settling_time_s']} "
            f"| {result['final_position_m']} "
            f"| {result['final_error_m']} "
            f"| {result['overshoot']} |",
            file=md,
        )

print(f"Saved to {markdown_path}")


# _, axs = plt.subplots(nrows=2, ncols=1, figsize=(10, 8))

# axs[0].yaxis.set_major_formatter(FormatStrFormatter("%.5f"))
# axs[1].yaxis.set_major_formatter(FormatStrFormatter("%.5f"))

# axs[0].set_title(f"Pelvis Position vs Desired Pelvis Position Gain K_p = {controller.walker.k_p}")
# axs[0].set_ylabel("meters (m)")
# axs[0].set_xlabel("time (s)")
# axs[0].axhline(
#     y=controller.x_d,
#     color="red",
#     linestyle="--",
#     label="Desired position",
# )
# axs[0].plot(current_time_history,average_position_history,color="red",label="Pelvis Position")
# axs[0].set_ylabel("meters (m)")
# axs[0].set_xlabel("time (s)")
# axs[0].grid(True)
# axs[0].legend() 


# axs[1].plot(current_time_history,commanded_velocity_linear_history,color="red",label="Velocity History")
# axs[1].grid(True)
# axs[1].legend() 

# plt.tight_layout()
# plt.show()