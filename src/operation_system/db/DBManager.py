from abc import ABC, abstractmethod

class DBManager(ABC):

    @abstractmethod
    def process_request(self, request):
        pass
