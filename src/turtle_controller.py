import math
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from custom_types import Point
from path_controller import PathController

from modules.publishers import Velocity
from modules.subscribers import Position, EulerData, Lidar


class TurtleBotController(Node):
    __euler_data = EulerData()
    __lidar_data = LaserScan()

    class ControllerUtils:
        @staticmethod
        def calculate_distance(point_a: Point, point_b: Point) -> float:
            return ((point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2) ** 0.5

        @staticmethod
        def calculate_angle(point_a: Point, point_b: Point) -> float:
            angle = math.atan2(point_b.y - point_a.y, point_b.x - point_a.x)
            return angle

        @staticmethod
        def clamp(value: float, min_value: float, max_value: float) -> float:
            return max(min_value, min(value, max_value))

    def __init__(self, path_controller: PathController):
        super().__init__("turtlebot_gazebo_controller")

        self.path_controller = path_controller
        self.velocity_module = Velocity(self)
        self.position_module = Position(self, self.__position_callback)
        self.lidar_module = Lidar(self, self.__lidar_callback)

        self.create_timer(0.16, self.__runtime)

    def __position_callback(self, euler_data: EulerData):
        self.__euler_data = euler_data

    def __lidar_callback(self, lidar_data: LaserScan):
        self.__lidar_data = lidar_data

    def __calculate_velocity_to_point(self, point: Point) -> Vector3:
        current_point = Point.from_euler_data(self.__euler_data)
        distance = self.ControllerUtils.calculate_distance(current_point, point)
        angle = self.ControllerUtils.calculate_angle(current_point, point)

        return Vector3(x=self.ControllerUtils.clamp(distance, 0.1, 0.5), y=0.0, z=angle)


    def __runtime(self):
        current_objective = self.path_controller.get_current_point()

        if current_objective is None:
            self.get_logger().info("Path is finished!")
            self.velocity_module.apply(Vector3(x=0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0))
            return

        velocity = self.__calculate_velocity_to_point(self.path_controller.get_current_point())
        angle_diff = self.__euler_data.yaw - velocity.z

        self.get_logger().info(f"Current point:\t {current_objective}")
        self.get_logger().info(f"Current position:\t {Point.from_euler_data(self.__euler_data)}")
        self.get_logger().info(f"Current velocity:\t {velocity}")
        self.get_logger().info(f"Current angle_diff:\t {angle_diff}")

        if abs(angle_diff) > 0.1:
            direction = -1 if angle_diff > 0 else 1
            self.velocity_module.apply(Vector3(x=0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=min(abs(angle_diff * 2.0), 1.0) * direction))
            return

        self.velocity_module.apply(Vector3(x=velocity.x, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0))

        if self.path_controller.is_point_reached(Point.from_euler_data(self.__euler_data)):
            self.velocity_module.apply(Vector3(x=0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0))
            self.path_controller.get_next_point()
            self.get_logger().info(f"Next point: {self.path_controller.get_current_point()}")