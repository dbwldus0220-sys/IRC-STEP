#!/usr/bin/env python3
"""Lightweight Gazebo transport helpers for STEP leg joint commands."""

import time


LEG_JOINT_TOPICS = {
    "RL0_wrap": "/step/right_hip_yaw_joint/cmd_pos",
    "RL1_wrap": "/step/right_hip_roll_joint/cmd_pos",
    "RL2_wrap": "/step/right_hip_pitch_joint/cmd_pos",
    "RL3_wrap": "/step/right_knee_pitch_joint/cmd_pos",
    "RL4_wrap": "/step/right_ankle_pitch_joint/cmd_pos",
    "RL5_wrap": "/step/right_ankle_roll_joint/cmd_pos",
    "LL0_wrap": "/step/left_hip_yaw_joint/cmd_pos",
    "LL1_wrap": "/step/left_hip_roll_joint/cmd_pos",
    "LL2_wrap": "/step/left_hip_pitch_joint/cmd_pos",
    "LL3_wrap": "/step/left_knee_pitch_joint/cmd_pos",
    "LL4_wrap": "/step/left_ankle_pitch_joint/cmd_pos",
    "LL5_wrap": "/step/left_ankle_roll_joint/cmd_pos",
}


class GazeboDoublePublishers:
    def __init__(self, topics, dry_run: bool, optional_topics=None):
        self.dry_run = dry_run
        self.node = None
        self.publishers = {}
        self.double_message_type = None
        self.optional_topics = set(optional_topics or ())
        self.warned_publish_topics = set()

        if dry_run:
            return

        try:
            from gz.msgs10.double_pb2 import Double
            from gz.transport13 import Node
        except ImportError as error:
            raise RuntimeError(
                "Gazebo Python transport bindings are required for replay: "
                "gz.transport13 and gz.msgs10"
            ) from error

        self.double_message_type = Double
        self.node = Node()
        for topic in topics:
            try:
                publisher = self.node.advertise(topic, Double)
            except Exception as error:  # Gazebo bindings expose runtime errors.
                if topic in self.optional_topics:
                    print(
                        f"[WARNING] Could not advertise optional Gazebo "
                        f"topic {topic}: {error}"
                    )
                    continue
                raise
            if not publisher.valid():
                if topic in self.optional_topics:
                    print(
                        f"[WARNING] Failed to advertise optional Gazebo "
                        f"topic: {topic}"
                    )
                    continue
                raise RuntimeError(f"Failed to advertise Gazebo topic: {topic}")
            self.publishers[topic] = publisher

        connection_deadline = time.monotonic() + 2.0
        while time.monotonic() < connection_deadline:
            if all(
                publisher.has_connections()
                for publisher in self.publishers.values()
            ):
                break
            time.sleep(0.01)

        disconnected_topics = [
            topic
            for topic, publisher in self.publishers.items()
            if not publisher.has_connections()
        ]
        disconnected_required_topics = [
            topic
            for topic in disconnected_topics
            if topic not in self.optional_topics
        ]
        for topic in disconnected_topics:
            if topic in self.optional_topics:
                print(
                    f"[WARNING] No Gazebo controller subscription found for "
                    f"optional topic: {topic}"
                )
        if disconnected_required_topics:
            missing_topics = ", ".join(disconnected_required_topics)
            raise RuntimeError(
                "No Gazebo controller subscription found for topic(s): "
                f"{missing_topics}"
            )

    def publish(self, commands) -> None:
        if self.dry_run:
            return

        for _, topic, _, command in commands:
            if topic not in self.publishers:
                continue
            message = self.double_message_type(data=command)
            try:
                published = self.publishers[topic].publish(message)
            except Exception as error:  # Keep an optional diagnostic lock safe.
                if topic in self.optional_topics:
                    if topic not in self.warned_publish_topics:
                        print(
                            f"[WARNING] Failed to publish optional Gazebo "
                            f"topic {topic}: {error}"
                        )
                        self.warned_publish_topics.add(topic)
                    continue
                raise
            if published is False:
                if topic in self.optional_topics:
                    if topic not in self.warned_publish_topics:
                        print(
                            f"[WARNING] Failed to publish optional Gazebo "
                            f"topic: {topic}"
                        )
                        self.warned_publish_topics.add(topic)
                    continue
                raise RuntimeError(f"Failed to publish Gazebo topic: {topic}")
