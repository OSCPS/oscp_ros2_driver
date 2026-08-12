#!/usr/bin/env python3

import time

import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from oscp_msgs.msg import OscpRaw
from magcal_wrapper.action import MagnetometerCalibration
from magcal_wrapper_py.core import MagnetometerCalibrator


class MagnetometerCalibrationNode(Node):

    def __init__(self):
        super().__init__("magnetometer_calibration_node")

        self.callback_group = ReentrantCallbackGroup()

        # Number of samples required before calibration.
        self.target_samples = 30000

        self.calibrating = False

        self.calibrator = MagnetometerCalibrator(min_samples=self.target_samples)

        # /oscp/raw is published with BEST_EFFORT reliability.
        raw_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10,)

        self.raw_subscription = self.create_subscription(OscpRaw,"/oscp/raw",self.raw_callback,raw_qos,callback_group=self.callback_group,)
        self.action_server = ActionServer(self,MagnetometerCalibration,"/DEV/oscp/magnetometer_calibration",execute_callback=self.execute_callback,goal_callback=self.goal_callback,cancel_callback=self.cancel_callback,callback_group=self.callback_group,)
        self.get_logger().info("Magnetometer calibration node started")
        self.get_logger().info("Listening on /oscp/raw")

    def goal_callback(self, goal_request):
        self.get_logger().info("Magnetometer calibration request received")

        if self.calibrating:
            self.get_logger().warn("Calibration already running")
            return GoalResponse.REJECT

        if not goal_request.start:
            self.get_logger().warn("Calibration goal requested with start=False")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Calibration cancellation requested")

        return CancelResponse.ACCEPT

    def raw_callback(self, msg):
        """
        Collect magnetometer samples from /oscp/raw.
        """

        if not self.calibrating:
            return

        if self.calibrator.sample_count >= self.target_samples:
            return

        try:
            self.calibrator.add_sample(msg.mag_x,msg.mag_y,msg.mag_z,)

        except ValueError as exc:
            self.get_logger().warn(f"Rejected magnetometer sample: {exc}")

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting magnetometer calibration")

        self.calibrator.reset()
        self.calibrating = True

        feedback = MagnetometerCalibration.Feedback()

        try:
            while not self.calibrator.ready:

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Calibration cancelled")
                    self.calibrating = False
                    goal_handle.canceled()
                    result = MagnetometerCalibration.Result()
                    result.success = False
                    return result

                sample_count = self.calibrator.sample_count

                feedback.progress = (100.0 * sample_count / self.target_samples)
                feedback.status = (f"Collecting samples "f"({sample_count}/{self.target_samples})")

                goal_handle.publish_feedback(feedback)

                # The MultiThreadedExecutor allows the /oscp/raw
                # callback to continue running on another thread.
                time.sleep(0.01)

            self.get_logger().info(f"Collected "f"{self.calibrator.sample_count}/"f"{self.target_samples} samples")

            # Diagnostic information BEFORE calibration
            # Access the raw samples stored by MagnetometerCalibrator.
            data = np.asarray(self.calibrator._samples,dtype=np.float64,)

            self.get_logger().info("========================================")
            self.get_logger().info("MAGNETOMETER DATA")
            self.get_logger().info("========================================")

            self.get_logger().info(f"Samples: {len(data)}")

            self.get_logger().info(f"Min: "f"[{np.min(data[:, 0]):.6f}, "f"{np.min(data[:, 1]):.6f}, "f"{np.min(data[:, 2]):.6f}]")

            self.get_logger().info(f"Max: "f"[{np.max(data[:, 0]):.6f}, "f"{np.max(data[:, 1]):.6f}, "f"{np.max(data[:, 2]):.6f}]")

            self.get_logger().info(f"Mean: "f"[{np.mean(data[:, 0]):.6f}, "f"{np.mean(data[:, 1]):.6f}, "f"{np.mean(data[:, 2]):.6f}]")

            self.get_logger().info(f"Std: "f"[{np.std(data[:, 0]):.6f}, "f"{np.std(data[:, 1]):.6f}, "f"{np.std(data[:, 2]):.6f}]")

            magnitudes = np.linalg.norm(data,axis=1,)

            self.get_logger().info(f"Magnitude min: "f"{np.min(magnitudes):.6f}")
            self.get_logger().info(f"Magnitude max: "f"{np.max(magnitudes):.6f}")
            self.get_logger().info(f"Magnitude mean: "f"{np.mean(magnitudes):.6f}")
            self.get_logger().info(f"Magnitude std: "f"{np.std(magnitudes):.6f}")
            self.get_logger().info("========================================")

            # ---------------------------------------------------------
            # Perform ellipsoid calibration
            # ---------------------------------------------------------

            feedback.progress = 100.0
            feedback.status = "Calculating calibration"

            goal_handle.publish_feedback(feedback)

            calibration = self.calibrator.calibrate()

            # ---------------------------------------------------------
            # Build action result
            # ---------------------------------------------------------

            result = MagnetometerCalibration.Result()
            result.success = True

            result.hard_iron_x = float(calibration.hard_iron[0])
            result.hard_iron_y = float(calibration.hard_iron[1])
            result.hard_iron_z = float(calibration.hard_iron[2])

            result.soft_iron = [
                float(calibration.soft_iron[0, 0]),
                float(calibration.soft_iron[0, 1]),
                float(calibration.soft_iron[0, 2]),

                float(calibration.soft_iron[1, 0]),
                float(calibration.soft_iron[1, 1]),
                float(calibration.soft_iron[1, 2]),

                float(calibration.soft_iron[2, 0]),
                float(calibration.soft_iron[2, 1]),
                float(calibration.soft_iron[2, 2]),
            ]
            result.fit_error = float(calibration.fit_error)

            # Log calibration result

            goal_handle.succeed()

            self.get_logger().info("========================================")
            self.get_logger().info("MAGNETOMETER CALIBRATION COMPLETED")
            self.get_logger().info("========================================")

            self.get_logger().info(f"Hard iron: "f"[{calibration.hard_iron[0]:.6f}, "f"{calibration.hard_iron[1]:.6f}, "f"{calibration.hard_iron[2]:.6f}]")

            self.get_logger().info("Soft iron:")

            self.get_logger().info(f"  [{calibration.soft_iron[0, 0]: .6f}, "f"{calibration.soft_iron[0, 1]: .6f}, "f"{calibration.soft_iron[0, 2]: .6f}]")
            self.get_logger().info(f"  [{calibration.soft_iron[1, 0]: .6f}, "f"{calibration.soft_iron[1, 1]: .6f}, "f"{calibration.soft_iron[1, 2]: .6f}]")
            self.get_logger().info(f"  [{calibration.soft_iron[2, 0]: .6f}, "f"{calibration.soft_iron[2, 1]: .6f}, "f"{calibration.soft_iron[2, 2]: .6f}]")

            self.get_logger().info(f"Fit error: "f"{calibration.fit_error:.6f}")
            self.get_logger().info(f"Mean radius: "f"{calibration.mean_radius:.6f}")
            self.get_logger().info(f"Std radius: "f"{calibration.std_radius:.6f}")
            self.get_logger().info("Calibration calculated only. Nothing has been written to the IMU.")
            self.get_logger().info("========================================")

            return result

        except Exception as exc:

            self.get_logger().error("Magnetometer calibration failed: "f"{type(exc).__name__}: {exc}")
            if goal_handle.is_active:
                goal_handle.abort()

            result = MagnetometerCalibration.Result()
            result.success = False

            return result

        finally:
            self.calibrating = False
            self.get_logger().info("Calibration session ended")


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerCalibrationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()