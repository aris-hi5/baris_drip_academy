from src.operation_system.db import DBManager
from src.operation_system.ui import UIManager


class ControlManager:
    def __init__(self, ui_manager: UIManager, db_manager: DBManager):
        self.ui_manager = ui_manager
        self.db_manager = db_manager

    def request_order(self):
        order = self.ui_manager.receive_order()
        commands = self.db_manager.get_all_commands()

        for command in commands:
            print(f"Processing command: {command}")
            # Baris와의 실제 상호작용 로직이 여기에 추가될 수 있습니다

        self.ui_manager.send_response({"status": "completed"})

    def return_status(self):
        status = self.ui_manager.receive_status()
        self.ui_manager.display_status(status)
