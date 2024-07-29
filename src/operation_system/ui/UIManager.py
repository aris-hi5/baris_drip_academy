from abc import ABC, abstractmethod

class UIManager(ABC):

    @abstractmethod
    def process_command(self, command):
        pass
