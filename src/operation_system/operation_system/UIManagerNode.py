import rclpy
from rclpy.node import Node
from library.Constants import Service, Constants
from message.srv import RobotService
from operation_system.UIManager import UIManager


class UIManagerNode(Node):
    def __init__(self):
        super().__init__("ui_manager_node")
        self.ui_manager = UIManager()
        self.cli = self.create_client(RobotService, Service.SERVICE_ROBOT)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = RobotService.Request()

    def send_request(self, command):
        self.req.cmd = self.ui_manager.process_command(command)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    ui_node = UIManagerNode()
    response = ui_node.send_request("YOUR_COMMAND_HERE")
    ui_node.get_logger().info(f"Response: {response}")
    ui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
