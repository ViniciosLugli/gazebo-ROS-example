import rclpy
from turtle_controller import TurtleBotController
from path_controller import PathController


def main(args=None):
    rclpy.init()

    path_controller = PathController('artifacts/path.csv')
    turtle_controller = TurtleBotController(path_controller)
    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()