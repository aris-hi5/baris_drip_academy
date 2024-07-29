import rclpy
from rclpy.node import Node
from library.Constants import Service, Topic, Constants
from message.srv import RobotService
from message.msg import ControlStatus
from .BarisManager import BarisManager

class BarisManagerNode(Node):
    def __init__(self):
        super().__init__('baris_manager_node')
        self.baris_manager = BarisManager()
        self.cli = self.create_client(RobotService, Service.SERVICE_ROBOT)
        self.publisher = self.create_publisher(ControlStatus, Topic.CONTROL_STATUS, 10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RobotService.Request()

    def send_request(self, command):
        processed_command = self.baris_manager.process_command(command)
        self.req.cmd = processed_command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def publish_status(self, status):
        processed_status = self.baris_manager.process_status(status)
        msg = ControlStatus()
        msg.status = processed_status
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    baris_node = BarisManagerNode()
    response = baris_node.send_request("YOUR_COMMAND_HERE")
    baris_node.publish_status("STATUS_HERE")
    baris_node.get_logger().info(f'Response: {response}')
    baris_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
