#!/usr/bin/env python3
"""Print native Gazebo odometry angular velocity, especially world-frame wz."""

import argparse
import signal
import threading

from google.protobuf import symbol_database
from gz.msgs10 import (
    header_pb2,
    odometry_pb2,
    pose_pb2,
    quaternion_pb2,
    time_pb2,
    twist_pb2,
    vector3d_pb2,
)
from gz.transport13 import Node


DEFAULT_TOPIC = "/step/base_odometry"

_SYMBOLS = symbol_database.Default()
for _message_class in (
    time_pb2.Time,
    header_pb2.Header,
    header_pb2.Header.Map,
    vector3d_pb2.Vector3d,
    quaternion_pb2.Quaternion,
    pose_pb2.Pose,
    twist_pb2.Twist,
    odometry_pb2.Odometry,
):
    _SYMBOLS.RegisterMessage(_message_class)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument(
        "--count", type=int, default=0,
        help="stop after this many messages; zero waits until Ctrl-C",
    )
    args = parser.parse_args()
    if args.count < 0:
        parser.error("--count must be non-negative")
    return args


class OdometryMonitor:
    def __init__(self, topic, count, done):
        self.count_limit = count
        self.done = done
        self.received = 0
        self.node = Node()
        self.callback = self.on_odometry
        self.subscribe_return = self.node.subscribe(
            odometry_pb2.Odometry, topic, self.callback
        )
        if self.subscribe_return is False:
            raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        timestamp = stamp.sec + stamp.nsec * 1e-9
        angular = message.twist.angular
        self.received += 1
        print(
            f"timestamp={timestamp:.9f} "
            f"wx={angular.x:+.9f} wy={angular.y:+.9f} wz={angular.z:+.9f}",
            flush=True,
        )
        if self.count_limit and self.received >= self.count_limit:
            self.done.set()


def main():
    args = parse_args()
    done = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: done.set())
    monitor = OdometryMonitor(args.topic, args.count, done)
    print(f"topic={args.topic}")
    print(f"subscribe_return={monitor.subscribe_return}")
    print("Waiting for gz.msgs.Odometry; start or step Gazebo when ready.")
    done.wait()
    print(f"received={monitor.received}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
