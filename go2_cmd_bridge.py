#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


class Go2CmdBridge(Node):
    def __init__(self):
        super().__init__("go2_cmd_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_bridge")
        self.declare_parameter("network_interface", "enp4s0")
        self.declare_parameter("max_vx", 0.3)
        self.declare_parameter("max_vyaw", 0.6)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("balance_stand_on_start", False)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        network_interface = self.get_parameter("network_interface").value
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vyaw = float(self.get_parameter("max_vyaw").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        control_rate = float(self.get_parameter("control_rate").value)
        balance_stand_on_start = bool(self.get_parameter("balance_stand_on_start").value)

        self.latest_vx = 0.0
        self.latest_vyaw = 0.0
        self.last_cmd_time = None
        self.command_active = False

        try:
            self.get_logger().info(f"Initializing Unitree SDK on interface: {network_interface}")
            ChannelFactoryInitialize(0, network_interface)
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(5.0)
            self.sport_client.Init()
            if balance_stand_on_start:
                self.sport_client.BalanceStand()
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize Unitree SDK: {exc}")
            sys.exit(1)

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(f"Subscribed to {self.cmd_vel_topic}")
        self.get_logger().info(f"Velocity limits: vx={self.max_vx:.2f}, vyaw={self.max_vyaw:.2f}")

    def cmd_callback(self, msg):
        self.latest_vx = max(-self.max_vx, min(self.max_vx, float(msg.linear.x)))
        self.latest_vyaw = max(-self.max_vyaw, min(self.max_vyaw, float(msg.angular.z)))
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        if self.last_cmd_time is None:
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            if self.command_active:
                self.send_move(0.0, 0.0)
                self.command_active = False
            return

        self.send_move(self.latest_vx, self.latest_vyaw)
        self.command_active = abs(self.latest_vx) > 1e-3 or abs(self.latest_vyaw) > 1e-3

    def send_move(self, vx, vyaw):
        try:
            self.sport_client.Move(float(vx), 0.0, float(vyaw))
        except Exception as exc:
            self.get_logger().error(f"Failed to send Move command: {exc}")

    def stop(self):
        try:
            self.sport_client.Move(0.0, 0.0, 0.0)
            self.sport_client.StopMove()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Go2CmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
