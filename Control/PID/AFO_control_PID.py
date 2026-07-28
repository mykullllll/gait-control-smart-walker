import serial
import time
import threading

import message_filters
import numpy as np
import rclpy
from matplotlib import pyplot as plt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64

from AFO_PID import Cluster, main_loop


class walker_control_node(Node):
    def __init__(self):
        super().__init__("Adaptive_Frequency_Oscillator_Reverse")

        # Runtime command and operator-confirmation state
        self.current_scan = None
        self.encoder = None
        self.latest_wheel_velocity = 0.0
        self.awaiting_confirmation = False
        self.assist_confirmed = threading.Event()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.actual_publish_history = []
        self.actual_publish_time = []

        now = self.get_clock().now()
        self.last_scan_update = now
        self.last_encoder_update = now

        # ROS publishers
        self.pub_shutdown = self.create_publisher(Bool, "/shutdown", 1)
        self.pub_right_motor = self.create_publisher(
            Float64, "/right_wheel_velocity", 1
        )
        self.pub_left_motor = self.create_publisher(Float64, "/left_wheel_velocity", 1)

        # Synchronized ROS subscribers consume a temporally aligned
        # scan/encoder pair instead of two independently latest values.
        scan_sub = message_filters.Subscriber(
            self, LaserScan, "/scan_legs_filtered", qos_profile=qos_profile_sensor_data
        )
        encoder_sub = message_filters.Subscriber(
            self, JointState, "/encoder_data", qos_profile=1
        )

        scan_sub.registerCallback(self.scan_arrival_callback)
        encoder_sub.registerCallback(self.encoder_arrival_callback)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, encoder_sub], queue_size=20, slop=0.10,
        )
        self.ts.registerCallback(self.synced_callback)

        # Perception and control components
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "Controller engaged. Please Begin Walking Calibrating Your Walking Style."
        )
        self.get_logger().info("=" * 50)

        self.cluster = Cluster()
        self.main = main_loop(fs=6)

        # Fixed-rate callbacks
        self.timer = self.create_timer(1.0 / self.main.fs, self.control_loop_callback)
        self.motor_timer = self.create_timer(1.0 / 30.0, self.motor_publish_callback)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def scan_arrival_callback(self, scan_msg):
        self.last_scan_update = self.get_clock().now()

    def encoder_arrival_callback(self, encoder_msg):
        self.last_encoder_update = self.get_clock().now()

    def synced_callback(self, scan_msg, encoder_msg):
        self.current_scan = scan_msg
        self.encoder = encoder_msg

    # ------------------------------------------------------------------
    # Hardware state
    # ------------------------------------------------------------------
    def arm_hardware_callback(self):
        # self.arm_timer.cancel()
        if rclpy.ok():
            arm_msg = Bool(data=False)
            self.pub_shutdown.publish(arm_msg)
            self.get_logger().info(
                "Sent hardware unlock command to ESP32 firmware gates."
            )

    # Command 0 Velocity to Motors cancel callback
    def stop_motor(self):
        print("STOPPING MOTORS AND ENGAGING FIRMWARE LOCKOUT...")
        try:
            # self.arm_timer.cancel()
            self.timer.cancel()
            self.motor_timer.cancel()
            self.latest_wheel_velocity = 0.0
            stop = Float64(data=0.0)
            self.pub_left_motor.publish(stop)
            self.pub_right_motor.publish(stop)

            lock = Bool(data=True)
            self.pub_shutdown.publish(lock)
        except Exception as e:
            print(f"Could not broadcast stop frame: {e}")

    # ------------------------------------------------------------------
    # Operator-confirmation handshake
    # ------------------------------------------------------------------
    def _wait_for_assist_confirmation(self):
        input("Calibration complete. Press Enter to begin powered assist...")
        self.assist_confirmed.set()

    def _hold_while_awaiting_confirmation(self):
        """Keep motor output paused until the operator confirms assist."""
        if not self.awaiting_confirmation:
            return False

        self.latest_wheel_velocity = 0.0
        if not self.assist_confirmed.is_set():
            return True

        self.awaiting_confirmation = False
        self.pub_shutdown.publish(Bool(data=False))
        return False

    def _request_assist_confirmation(self):
        """Start the non-blocking operator-confirmation prompt."""
        self.awaiting_confirmation = True
        self.assist_confirmed.clear()
        threading.Thread(
            target=self._wait_for_assist_confirmation, daemon=True,
        ).start()
        self.latest_wheel_velocity = 0.0

    # ------------------------------------------------------------------
    # Fixed-rate motor output
    # ------------------------------------------------------------------
    def motor_publish_callback(self):
        """Publish the latest command, or zero it when sensor data is stale."""
        now = self.get_clock().now()

        scan_age = (now - self.last_scan_update).nanoseconds / 1e9

        encoder_age = (now - self.last_encoder_update).nanoseconds / 1e9

        stale = scan_age > 0.5 or encoder_age > 0.25

        velocity = 0.0 if stale else self.latest_wheel_velocity

        self.actual_publish_history.append(velocity)
        self.actual_publish_time.append(now.nanoseconds / 1e9 - self.start_time)

        if stale:
            self.get_logger().warning(
                "Motor command zeroed: "
                f"scan_age={scan_age:.3f}s, "
                f"encoder_age={encoder_age:.3f}s",
                throttle_duration_sec=0.5,
            )

        self.pub_left_motor.publish(Float64(data=velocity))
        self.pub_right_motor.publish(Float64(data=velocity))

    # ------------------------------------------------------------------
    # Perception and control loop
    # ------------------------------------------------------------------
    def control_loop_callback(self):
        # Stage 1: require both synchronized sensor streams.
        if self.current_scan is None:
            self.get_logger().info("no scan", throttle_duration_sec=1.0)
            return

        if self.encoder is None:
            self.get_logger().info("no encoder data", throttle_duration_sec=1.0)
            return

        try:
            # Stage 2: convert the latest sensor pair into a leg observation.
            encoder_velocity = (
                self.encoder.velocity[0] + self.encoder.velocity[1]
            ) / 2.0
            current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time

            collisions = self.cluster.process_scan(
                self.current_scan.angle_min,
                self.current_scan.angle_increment,
                self.current_scan.ranges,
                0,
            )
            raw_left, raw_right, isoccluded, shutdown = self.cluster.cluster_find(
                collisions
            )

            if shutdown is True:
                self.stop_motor()
                self.get_logger().error(
                    "Persistent leg occlusion. Stopping controller."
                )
                rclpy.shutdown()
                return

            if raw_left is None or raw_right is None:
                return

            # Stage 3: hold after calibration until the operator confirms.
            if self._hold_while_awaiting_confirmation():
                return

            # Stage 4: run one calibration/control step.
            was_calibrated = self.main.calibrated

            step_result = self.main.step_from_legs(
                current_time, encoder_velocity, raw_left[0], raw_right[0], isoccluded
            )

        except Exception as e:
            self.latest_wheel_velocity = 0.0
            self.get_logger().error(
                f"control_loop_callback failed; command zeroed: {e}",
                throttle_duration_sec=1.0,
            )
            return

        # Stage 5: pause on the one-time transition out of calibration.
        if not was_calibrated and self.main.calibrated:
            self._request_assist_confirmation()
            return

        # Stage 6: calibration can legitimately return no motor command.
        if step_result is None or step_result[0] is None:
            return

        # Stage 7: make the controller result available to the 30 Hz publisher.
        wheel_velocity, _ = step_result

        arm_msg = Bool(data=False)
        self.pub_shutdown.publish(arm_msg)
        self.latest_wheel_velocity = wheel_velocity


# ESP32 Hardware Reset on Shutdown
def pulse_esp32_reset():
    """Uses native serial lines with an explicit hardware settling delay to force an ESP32 reboot."""
    print("⚡ Flashing DTR/RTS lines to force firmware reboot...")
    try:
        # Open the serial hook cleanly
        ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

        # 1. Drive the EN pin LOW by setting DTR/RTS states
        ser.setDTR(False)
        ser.setRTS(True)

        # 🔬 THE CAPACITOR DRAIN BUFFER
        # 50 milliseconds is the sweet spot: long enough to completely discharge
        # the onboard RC filter circuit on the EN pin, but fast enough that you won't feel the lag.
        time.sleep(0.05)

        # 2. Release the lines back to high to allow the chip to boot into a clean state
        ser.setDTR(True)
        ser.setRTS(False)

        ser.close()
        print("✅ ESP32 hardware reset successful. Motors locked.")
    except Exception as e:
        print(f"Could not signal hardware lines: {e}")


def main(args=None):
    rclpy.init(args=args)
    walker_node = walker_control_node()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(walker_node)
    except KeyboardInterrupt:
        walker_node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        # Stop publishing velocity commands immediately
        walker_node.stop_motor()

        # Stops running Control Loop Callback
        walker_node.timer.cancel()

        # ESP32 Connection to Motors Locked
        if hasattr(walker_node, "arm_timer"):
            walker_node.arm_timer.cancel()

        # Let the final frames exit the socket stack cleanly
        if rclpy.ok():
            rclpy.spin_once(walker_node, timeout_sec=0.2)

        walker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        pulse_esp32_reset()
        # Safe shutdown procedure

        if (
            hasattr(walker_node, "main")
            and len(walker_node.main.commanded_timestamps) > 0
        ):

            velocity_history = np.array(walker_node.main.commanded_velocity_history)
            encoder_velocity = np.array(walker_node.main.encoder_data)

            start_idx = len(walker_node.main.encoder_data) - len(
                walker_node.main.commanded_velocity_history
            )
            error = velocity_history - encoder_velocity[start_idx:]
            mean_error = np.mean(error)
            mae = np.mean(np.abs(error))
            rmse = np.sqrt(np.mean(np.square(error)))
            command_std = np.std(velocity_history)

            print("\n Generating Post-Run Gait Calibration Plots...")

            print(f"Calibration time: {walker_node.main.time_to_cal} s")
            print(
                f"Mean Command: {np.nanmean(walker_node.main.commanded_velocity_history):.3f} rad/s"
            )
            print(
                f"Mean Encoder:      {np.nanmean(walker_node.main.encoder_data):.3f} rad/s"
            )
            print(f"RMSE Predicted to True Velocity:       {rmse:.3f} rad/s")
            print(
                f"Mean error: {mean_error} --- Negative : missing low --- Positive : missing high ---"
            )
            print(f"Mean error absolute (MAE): {mae}")
            print(f"Standard Deviation of commanded velocity {command_std}")
            print(
                f"Time in Ramping {100*(walker_node.main.control_state.count(0)/len(walker_node.main.control_state))} % "
            )
            print(
                f"Time in Tracking: {100*(walker_node.main.control_state.count(1)/len(walker_node.main.control_state))} %"
            )
            print(
                f"Time in Frozen: {100*(walker_node.main.control_state.count(2)/len(walker_node.main.control_state))} %"
            )
            print(f"Time detected Frozen Gait {walker_node.main.freeze_event_history}")

            _, axs = plt.subplots(nrows=3, ncols=2, figsize=(10, 8))

            axs[0, 0].plot(
                walker_node.main.commanded_timestamps,
                walker_node.main.commanded_velocity_history,
                color="red",
                linestyle="--",
                label="Velocity Command",
            )
            axs[0, 0].plot(
                walker_node.main.encoder_time,
                walker_node.main.encoder_data,
                color="blue",
                label="Encoder Data Feedback",
            )
            axs[0, 0].set_title("Velocity Command vs Encoder Data")
            axs[0, 0].set_ylabel("rad/s")
            axs[0, 0].set_xlabel("Time (s)")
            axs[0, 0].legend()
            axs[0, 0].grid(True)

            axs[0, 1].plot(
                walker_node.main.commanded_timestamps,
                walker_node.main.cadence_history,
                color="red",
                linestyle="--",
                label="Cadence (Hz)",
            )
            axs[0, 1].set_ylabel("Hertz (Hz)")
            axs[0, 1].set_xlabel("Time (s)")
            axs[0, 1].set_title("Cadence History (Hz)")
            axs[0, 1].legend()
            axs[0, 1].grid(True)

            axs[1, 0].plot(
                walker_node.main.encoder_time,
                walker_node.main.pelvis_history,
                color="red",
                linestyle="--",
                label="Pelvis Position (m)",
            )
            axs[1, 0].set_ylabel("meters (m)")
            axs[1, 0].set_xlabel("Time (s)")
            axs[1, 0].set_title("Pelvis Position (m)")
            axs[1, 0].legend()
            axs[1, 0].grid(True)

            axs[1, 1].plot(
                walker_node.main.commanded_timestamps,
                walker_node.main.control_state,
                color="purple",
                linestyle="--",
                label="State of Control",
            )
            axs[1, 1].set_title("Control State")
            axs[1, 1].set_ylabel("Control State")
            axs[1, 1].set_xlabel("Time (s)")
            axs[1, 1].set_yticks([0, 1, 2])
            axs[1, 1].set_yticklabels(["Ramping", "Tracking", "Frozen"])
            axs[1, 1].legend()
            axs[1, 1].grid(True)

            axs[2, 0].plot(
                walker_node.main.commanded_timestamps,
                walker_node.main.stride_used_history,
                label="Stride Used",
            )
            axs[2, 0].set_title("Stride Used")
            axs[2, 0].set_ylabel("m")
            axs[2, 0].grid(True)
            axs[2, 0].legend()

            axs[2, 1].plot(
                walker_node.main.commanded_timestamps,
                walker_node.main.target_wheel_history,
                label="Target Wheel Velocity",
            )
            axs[2, 1].plot(
                walker_node.actual_publish_time,
                walker_node.actual_publish_history,
                label="Actually Published",
                alpha=0.8,
            )
            axs[2, 1].set_title("Target vs Published Command")
            axs[2, 1].set_ylabel("rad/s")
            axs[2, 1].grid(True)
            axs[2, 1].legend()

            plt.tight_layout()
            plt.show()

        else:
            print("\n No gait telemetry data was captured to plot.")


if __name__ == "__main__":
    main()
