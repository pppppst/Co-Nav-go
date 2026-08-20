#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudInspector(Node):
    def __init__(self, topic, max_points, reliability):
        super().__init__("pointcloud2_inspector")
        self.topic = topic
        self.max_points = max_points
        self.done = False
        qos = QoSProfile(depth=10)
        if reliability == "reliable":
            qos.reliability = ReliabilityPolicy.RELIABLE
        elif reliability == "best_effort":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud, qos)
        self.get_logger().info(
            f"Waiting for one PointCloud2 message on {topic} "
            f"(reliability={reliability})"
        )

    def on_cloud(self, msg):
        field_names = [field.name for field in msg.fields]
        print(f"header.stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"header.frame_id: {msg.header.frame_id}")
        print(f"width x height: {msg.width} x {msg.height}")
        print(f"point_step: {msg.point_step}")
        print("fields:")
        for field in msg.fields:
            print(
                f"  {field.name}: offset={field.offset} "
                f"datatype={field.datatype} count={field.count}"
            )

        wanted = [
            name
            for name in ["x", "y", "z", "intensity", "ring", "time", "t", "timestamp"]
            if name in field_names
        ]
        if not wanted:
            print("No standard fields found.")
            self.done = True
            return

        stats = {name: {"min": math.inf, "max": -math.inf, "count": 0} for name in wanted}
        first_rows = []
        for i, point in enumerate(point_cloud2.read_points(msg, field_names=wanted, skip_nans=True)):
            if i < 8:
                first_rows.append(point)
            for name, value in zip(wanted, point):
                if isinstance(value, float) and not math.isfinite(value):
                    continue
                stats[name]["min"] = min(stats[name]["min"], float(value))
                stats[name]["max"] = max(stats[name]["max"], float(value))
                stats[name]["count"] += 1
            if i + 1 >= self.max_points:
                break

        print("first points:")
        for row in first_rows:
            print(f"  {dict(zip(wanted, row))}")

        print("stats:")
        for name, values in stats.items():
            print(
                f"  {name}: min={values['min']:.9g} "
                f"max={values['max']:.9g} count={values['count']}"
            )

        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/utlidar/cloud")
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument(
        "--reliability",
        choices=["reliable", "best_effort", "system_default"],
        default="best_effort",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudInspector(args.topic, args.max_points, args.reliability)
    start_time = node.get_clock().now()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed >= args.timeout:
            node.get_logger().error(
                f"Timed out after {args.timeout:.1f}s waiting for {args.topic}. "
                "Check ROS_DOMAIN_ID/RMW env and try --reliability reliable."
            )
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
