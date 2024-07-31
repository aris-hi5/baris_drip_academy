import sqlite3


class DBManager:
    def __init__(self, db_path):
        self.connection = sqlite3.connect(db_path)
        self.cursor = self.connection.cursor()
        self.initialize_database()
        self.before_pos = "cmd"

    """
    데이터베이스 초기화
    """

    def initialize_database(self):
        create_baris_commands_table = """
        CREATE TABLE IF NOT EXISTS baris_commands (
            no INTEGER PRIMARY KEY,
            command TEXT NOT NULL,
            par1 TEXT,
            par2 TEXT,
            par3 TEXT,
            par4 TEXT,
            par5 TEXT
        )
        """
        self.process_request(create_baris_commands_table)
        self.insert_initial_data()

    """
    초기 데이터 삽입
    """

    def insert_initial_data(self):
        # Initial data insertion with comments
        insert_data_query = """
        INSERT OR IGNORE INTO baris_commands (no, command, par1, par2, par3, par4, par5) VALUES
        (1, 'HOME_NORMAL', '0', '0', '0', '0', '0'), -- 로봇 기본 위치
        (2, 'GRIPPER_INIT', '0', '0', '0', '0', '0'), -- 그리퍼 활성화
        (3, 'PICKUP', 'DSP', '2', '0', '0', '0'), -- 드리퍼 잡기
        (4, 'HOLD', 'COF', '0', '0', '0', '0'), -- 커피 머신으로 가기
        (5, 'COFFEE_ON', '0', '0', '0', '0', '0'), -- 커피 동작 - 종료
        (6, 'UNHOLD', 'COF', '0', '0', '0', '0'), -- 커피 머신에서 나오기
        (7, 'HOME_NORMAL', '0', '0', '0', '0', '0'), -- 로봇 기본 위치
        (8, 'FLATTENING', 'ZON', '1', '0', '0', '0'), -- 커피 흔들기
        (9, 'PLACE_DRIP', 'ZON', '1', '0', '0', '0'), -- 드리퍼 드립존에 놓기
        (10, 'PICKUP', 'DSP', '1', '0', '0', '0'), -- 컵 잡기
        (11, 'PLACE_CUP', 'ZON', '1', '0', '0', '0'), -- 컵 드립존에 놓기
        (12, 'PICKUP', 'KET', '0', '0', '0', '0'), -- 주전자 잡으러 가기
        (13, 'WATER_TOGGLE', '0', '0', '0', '0', '0'), -- 모아이 동작 - 종료
        (14, 'DRAIN_FIT', 'HOT', '1', 'DP', '0', '0'), -- 주전자 안에 있는 물양 맞추기
        (15, 'HOME_KETTLE', '0', '0', '0', '0', '0'), -- 주전자 잡은 로봇 기본 위치
        (16, 'DRIP', 'DP', 'SOL', 'HOT', '1', '1'), -- 드립하기
        (17, 'HOME_KETTLE', '0', '0', '0', '0', '0'), -- 주전자 잡은 로봇 기본 위치
        (18, 'DRAIN_ALL', '0', '0', '0', '0', '0'), -- 주전안에 남은 물 다 버리기
        (19, 'PLACE', 'KET', '0', '0', '0', '0'), -- 주전자 내려놓기
        (20, 'HOME_NORMAL', '0', '0', '0', '0', '0'), -- 로봇 기본 위치
        (21, 'PICKUP_CUP', 'ZON', '1', '0', '0', '0'), -- 드립존에 있는 컵 잡기
        (22, 'PLACE', 'PIC', '1', '0', '0', '0'), -- 컵 드립존에 놓기
        (23, 'PICKUP_DRIP', 'ZON', '1', '0', '0', '0'), -- 드립존에 있는 드리퍼 들기
        (24, 'PLACE', 'BIN', '0', '0', '0', '0'), -- 드리퍼 쓰레기통에 버리기
        (25, 'GESTURE', 'ETC', '2', '0', '0', '0') -- 인사
        """
        self.process_request(insert_data_query)

    """
    명령을 no로 조회
    """

    def get_command_by_no(self, no) -> list:
        try:
            self.cursor.execute(f"SELECT * FROM baris_commands WHERE no={no}")
            result = self.cursor.fetchall()
            resultDetail = result[0]
            return (
                resultDetail[1],
                resultDetail[2],
                resultDetail[3],
                resultDetail[4],
                resultDetail[5],
                resultDetail[6],
            )
        except sqlite3.Error as e:
            print(f"SQLite error: {e}")

    """
    명령 실행
    """

    def process_request(self, request):
        try:
            self.cursor.execute(request)
            self.connection.commit()
            return True
        except sqlite3.Error as e:
            print(f"SQLite error: {e}")
            return False

    def get_cur_cmd(self):
        return self.before_pos


def main():
    # 데이터베이스 초기화 (테스트용)
    db_path = "test.db"
    manager = DBManager(db_path)

    # get_all_commands 테스트
    print("All commands in baris_commands table:")
    for i in range(1, 26):
        # message = manager.get_command_by_no(i)
        print(manager.get_command_by_no(i))


if __name__ == "__main__":
    main()
