import rclpy
from rclpy.node import Node
from library.Constants import Service, Constants
from message.srv import DBService
from src.operation_system.db.DBManager import DBManager


class DBManagerNode(Node):
    def __init__(self):
        super().__init__("db_manager_node")
        self.db_manager = DBManager("test.db")
        self.srv = self.create_service(DBService, Service.SERVICE_DB, self.db_callback)

    def db_callback(self, request, response):
        response.success = self.db_manager.process_request(request.sql_query)
        return response


def main(args=None):
    rclpy.init(args=args)
    db_node = DBManagerNode()
    rclpy.spin(db_node)
    db_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
