import sqlite3
import rclpy
from rclpy.node import Node
from message.srv import RobotService, DispenseService, DBService
from message.msg import DispenserStatus
from library.Constants import Service, Topic

class Client(Node):

    def __init__(self):
        super().__init__('robot_dispenser_client')
        
        # RobotService 클라이언트 생성
        self.robot_client = self.create_client(RobotService, Service.SERVICE_ROBOT)
        while not self.robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for RobotService...')
        
        # DispenseService 클라이언트 생성
        self.dispense_client = self.create_client(DispenseService, Service.SERVICE_DISPENSER)
        while not self.dispense_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DispenseService...')
        
        self.db_client = self.create_client(DBService, Service.SERVICE_DB)
        while not self.db_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DBService...')
        
        # DispenserStatus 토픽 구독자 생성
        self.subscription = self.create_subscription(
            DispenserStatus,
            Topic.ROBOT_STATUS,
            self.dispenser_status_callback,
            10)
        self.subscription  # 방지용: 구독자가 사라지지 않도록 유지

    def send_robot_request(self, seq_no, cmd, par1, par2, par3, par4, par5):
        request = RobotService.Request()
        request.seq_no = seq_no
        request.cmd = cmd
        request.par1 = par1
        request.par2 = par2
        request.par3 = par3
        request.par4 = par4
        request.par5 = par5
        
        self.future = self.robot_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_dispense_request(self, seq_no, cmd):
        request = DispenseService.Request()
        request.seq_no = seq_no
        request.command = cmd
        
        self.future = self.dispense_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def dispenser_status_callback(self, msg):
        self.get_logger().info(f'Received status update: seq_no={msg.seq_no}, node_status={msg.node_status}')
        for component in msg.component:
            self.get_logger().info(f'Component status: {component.status}, stock: {component.stock}, status_code: {component.status_code}')
    

    def send_DB_request(self, seq_no):
        request = DBService.Request()
        request.seq_no = seq_no
       
        self.future = self.db_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


# def fetch_commands_from_db(db_path):
#     conn = sqlite3.connect(db_path)
#     cursor = conn.cursor()
#     cursor.execute("SELECT seq_no, command, par1, par2, par3, par4, par5 FROM commands ORDER BY seq_no")
#     commands = cursor.fetchall()
#     conn.close()
#     return commands

def main(args=None):
    rclpy.init(args=args)
    client = Client()

    # db_path = 'path_to_your_database.db'
    # commands = fetch_commands_from_db(db_path)
    for i in range(1,26):
        response = client.send_DB_request(str(i))
        commands = [response.cmd, response.par1, response.par2, response.par3, response.par4, response.par5]

        if commands[0] in ['COFFEE_ON', 'WATER_TOGGLE']:
            response = client.send_dispense_request(str(i), commands[0])
        else:
            response = client.send_robot_request(str(i), commands[0], commands[1], commands[2], commands[3], commands[4], commands[5])
        client.get_logger().info(f'Response for {commands}: {response}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
