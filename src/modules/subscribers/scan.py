from .base import Subscription
from rclpy.node import Node
from typing import Any
from sensor_msgs.msg import LaserScan


class Lidar(Subscription):
    def __init__(self, node: Node, subscription_callback: Any):
        super().__init__("Lidar", node, f"/scan", LaserScan)

        super().connect(subscription_callback)