from .ControlManager import ControlManager

class BarisManager(ControlManager):
    def process_command(self, command):
        # Baris 관련 명령어 처리 로직
        return command

    def process_status(self, status):
        # Baris 관련 상태 처리 로직
        return status
