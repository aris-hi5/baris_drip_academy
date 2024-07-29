from abc import ABC, abstractmethod

class ControlManager(ABC):

    @abstractmethod
    def process_command(self, command):
        pass

    @abstractmethod
    def process_status(self, status):
        pass
