#!/usr/bin/env python3

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from magcal_wrapper.action import MagnetometerCalibration

class MagCalibrationClient(Node):

    def __init__(self):
        super().__init__(
            "mag_calibration_client"
        )

        self._action_client = ActionClient(
            self,
            MagnetometerCalibration,
            "/oscp/magnetometer_calibration",
        )

    def send_goal(self):

        goal = MagnetometerCalibration.Goal()
        goal.start = True

        self.get_logger().info(
            "Waiting for calibration server..."
        )

        self._action_client.wait_for_server()

        self.get_logger().info(
            "Sending calibration request..."
        )

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Calibration goal rejected"
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            "Calibration goal accepted"
        )

        future = goal_handle.get_result_async()

        future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f"{feedback.progress:.1f}% - "
            f"{feedback.status}"
        )

    def get_result_callback(self, future):

        result = future.result().result

        if not result.success:

            self.get_logger().error(
                "Magnetometer calibration failed"
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            "Magnetometer calibration succeeded!"
        )

        self.get_logger().info(
            "Hard-iron offset:"
        )

        self.get_logger().info(
            f"  X = {result.hard_iron_x:.6f}"
        )

        self.get_logger().info(
            f"  Y = {result.hard_iron_y:.6f}"
        )

        self.get_logger().info(
            f"  Z = {result.hard_iron_z:.6f}"
        )

        self.get_logger().info(
            f"Fit error = {result.fit_error:.6f}"
        )

        self.get_logger().info(
            "Soft-iron correction matrix:"
        )

        matrix = result.soft_iron

        self.get_logger().info(
            f"  [{matrix[0]: .6f}, "
            f"{matrix[1]: .6f}, "
            f"{matrix[2]: .6f}]"
        )

        self.get_logger().info(
            f"  [{matrix[3]: .6f}, "
            f"{matrix[4]: .6f}, "
            f"{matrix[5]: .6f}]"
        )

        self.get_logger().info(
            f"  [{matrix[6]: .6f}, "
            f"{matrix[7]: .6f}, "
            f"{matrix[8]: .6f}]"
        )

        self.get_logger().info(
            "Calibration calculated only."
        )

        self.get_logger().info(
            "Nothing has been written to the IMU."
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    client = MagCalibrationClient()

    client.get_logger().info(
        "Rotate the IMU through as many orientations "
        "as possible while calibration is running."
    )

    client.send_goal()

    rclpy.spin(client)

    client.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()