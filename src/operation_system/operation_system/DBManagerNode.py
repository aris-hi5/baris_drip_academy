import rclpy as rp
from rclpy.node import Node
from library.Constants import Service, Constants
from message.srv import DBService
from operation_system.DBManager import DBManager


class DBManagerNode(Node):
    def __init__(self):
        super().__init__(Constants.DB_MANAGER_NODE)

        #   DBManagerNode 서비스 생성
        self.service = self.create_service(
            DBService, Service.SERVICE_DB, self.callback_db_manager
        )

        #   DBManager 객체 생성
        self.db_manager = DBManager("test.db")
        self.get_logger().info(f"Current Cmd {self.db_manager.get_cur_cmd()}")

    def callback_db_service(self, request, response):
        """
        DB 서비스 요청을 받으면 실행되는 메소드
        :param request: 서비스 콜을 받은 메세지
        :param response: 서비스 콜을 처리 후 전달할 메세지
        :return: 서비스 응답
        """
        try:
            return_value = self.db_manager.get_command_by_no(request.seq_no)
            response.cmd = return_value[0]
            response.par1 = return_value[1]
            response.par2 = return_value[2]
            response.par3 = return_value[3]
            response.par4 = return_value[4]
            response.par5 = return_value[5]

        except Exception as error:
            print(f"DBManagerNode callback_db_manager {error=}, {type(error)=}")

        finally:
            return response


def main(args=None):
    rp.init(args=args)
    node = DBManagerNode()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt (SIGINT)")

    except SystemExit:  # <--- process the exception
        print("DBManagerNode System Exiting")

    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == "__main__":
    main()
