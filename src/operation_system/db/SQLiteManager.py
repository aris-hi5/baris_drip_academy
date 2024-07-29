from .DBManager import DBManager
import sqlite3

class SQLiteManager(DBManager):

    def __init__(self, db_path):
        self.connection = sqlite3.connect(db_path)

    def process_request(self, request):
        # SQLite를 사용하여 DB 요청을 처리하는 로직
        return True
