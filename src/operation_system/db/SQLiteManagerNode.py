import rclpy
from rclpy.node import Node
from library.Constants import Service, Constants
from message.srv import DBService
from .SQLiteManager import SQLiteManager

class SQLiteManagerNode(Node):
    def __init__(self):
        super().__init__('sqlite_manager_node')
        self.db_manager = SQLiteManager('/path/to/your/database.db')
        self.srv = self.create_service(DBService, Service.SERVICE_DB, self.db_callback)

    def db_callback(self, request, response):
        response.success = self.db_manager.process_request(request)
        return response

def main(args=None):
    rclpy.init(args=args)
    db_node = SQLiteManagerNode()
    rclpy.spin(db_node)
    db_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
