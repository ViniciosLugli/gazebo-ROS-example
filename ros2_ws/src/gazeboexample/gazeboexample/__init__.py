from src import PathController, TurtleBotController
import rclpy

def main(args=None):
    rclpy.init()

    path_controller = PathController('artifacts/path.csv')
    turtle_controller = TurtleBotController(path_controller)
    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()