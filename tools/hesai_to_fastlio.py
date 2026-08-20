#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class HesaiToFastLio(Node):
    def __init__(self, input_topic, output_topic, reliability):
        super().__init__("hesai_to_fastlio")
        qos = QoSProfile(depth=5)
        if reliability == "reliable":
            qos.reliability = ReliabilityPolicy.RELIABLE
        elif reliability == "best_effort":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, qos)
        self.output_topic = output_topic
        self.get_logger().info(f"Converting {input_topic} -> {output_topic}")

    @staticmethod
    def get_value(row, name, index):
        try:
            return row[name]
        except (IndexError, KeyError, TypeError, ValueError):
            return row[index]

    def on_cloud(self, msg):
        fields = {field.name for field in msg.fields}
        required = {"x", "y", "z", "intensity", "ring"}
        missing = required - fields
        if missing:
            self.get_logger().error(f"Input cloud missing fields: {sorted(missing)}")
            return

        has_timestamp = "timestamp" in fields
        has_time = "time" in fields
        read_fields = ["x", "y", "z", "intensity", "ring"]
        if has_timestamp:
            read_fields.append("timestamp")
        elif has_time:
            read_fields.append("time")

        frame_start = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points = []
        for row in point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True):
            x = self.get_value(row, "x", 0)
            y = self.get_value(row, "y", 1)
            z = self.get_value(row, "z", 2)
            intensity = self.get_value(row, "intensity", 3)
            ring = self.get_value(row, "ring", 4)
            if has_timestamp:
                rel_time = float(self.get_value(row, "timestamp", 5)) - frame_start
            elif has_time:
                rel_time = float(self.get_value(row, "time", 5))
            else:
                rel_time = 0.0
            if not math.isfinite(rel_time) or rel_time < 0.0:
                rel_time = 0.0
            points.append((float(x), float(y), float(z), float(intensity), float(rel_time), int(ring)))

        out_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
        ]
        out = point_cloud2.create_cloud(msg.header, out_fields, points)
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="/lidar_points")
    parser.add_argument("--output", default="/lidar_points_fastlio")
    parser.add_argument(
        "--reliability",
        choices=["reliable", "best_effort", "system_default"],
        default="reliable",
    )
    args = parser.parse_args()

    rclpy.init()
    node = HesaiToFastLio(args.input, args.output, args.reliability)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
