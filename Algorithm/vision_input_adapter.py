#!/usr/bin/env python3

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionInputAdapter(Node):
    """
    Subscribe to Vision JSON topics and store only the latest valid values.

    This node does not decide robot motion.
    It only receives, parses, validates, and stores vision observations.
    """

    def __init__(self) -> None:
        super().__init__("vision_input_adapter")

        self.latest_ball: dict[str, Any] | None = None
        self.latest_line: dict[str, Any] | None = None
        self.latest_hurdle: dict[str, Any] | None = None

        self.ball_received_time = None
        self.line_received_time = None
        self.hurdle_received_time = None

        self.create_subscription(
            String,
            "/vision/ball_info",
            self.ball_callback,
            10,
        )

        self.create_subscription(
            String,
            "/vision/line_info",
            self.line_callback,
            10,
        )

        self.create_subscription(
            String,
            "/vision/hurdle_info",
            self.hurdle_callback,
            10,
        )

        self.status_timer = self.create_timer(
            1.0,
            self.print_input_status,
        )

        self.get_logger().info(
            "Vision input adapter started"
        )

    def parse_json_message(
        self,
        message: String,
        source_name: str,
    ) -> dict[str, Any] | None:
        try:
            data = json.loads(message.data)

        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"{source_name}: invalid JSON: {exc}"
            )
            return None

        if not isinstance(data, dict):
            self.get_logger().warning(
                f"{source_name}: JSON root must be an object"
            )
            return None

        return data

    def ball_callback(
        self,
        message: String,
    ) -> None:
        data = self.parse_json_message(
            message,
            "ball",
        )

        if data is None:
            return

        self.latest_ball = data
        self.ball_received_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            "BALL RECEIVED: "
            f"detected={data.get('detected')}"
        )

    def line_callback(
        self,
        message: String,
    ) -> None:
        data = self.parse_json_message(
            message,
            "line",
        )

        if data is None:
            return

        self.latest_line = data
        self.line_received_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            "LINE RECEIVED: "
            f"detected={data.get('detected')}"
        )

    def hurdle_callback(
        self,
        message: String,
    ) -> None:
        data = self.parse_json_message(
            message,
            "hurdle",
        )

        if data is None:
            return

        self.latest_hurdle = data
        self.hurdle_received_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            "HURDLE RECEIVED: "
            f"detected={data.get('detected')}"
        )

    def is_data_fresh(
        self,
        received_time,
        timeout_sec: float = 0.5,
    ) -> bool:
        """
        Return True when the received vision data
        is recent enough.
        """

        if received_time is None:
            return False

        now = self.get_clock().now()

        age_sec = (
            now - received_time
        ).nanoseconds / 1_000_000_000.0

        return age_sec <= timeout_sec

    def print_input_status(self) -> None:
        ball_fresh = self.is_data_fresh(
            self.ball_received_time
        )

        line_fresh = self.is_data_fresh(
            self.line_received_time
        )

        hurdle_fresh = self.is_data_fresh(
            self.hurdle_received_time
        )

        self.get_logger().info(
            "VISION STATUS: "
            f"ball_fresh={ball_fresh}, "
            f"line_fresh={line_fresh}, "
            f"hurdle_fresh={hurdle_fresh}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = VisionInputAdapter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()