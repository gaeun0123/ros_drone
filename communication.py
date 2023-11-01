import time
from pymavlink import mavutil

# 데이터 구조체 정의
class DroneData:
    def __init__(self, id, x, y, z, velocity_x, velocity_y, velocity_z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
        self.velocity_z = velocity_z

# MAVLink 연결 설정
def connect_mavlink(port, baudrate=57600):
    return mavutil.mavlink_connection(port, baud=baudrate)

# 데이터 전송 함수
def send_data(mavlink_connection, drone_data):
    mavlink_connection.mav.global_position_int_send(
        drone_data.id,
        int(drone_data.x * 1e7),  # latitude
        int(drone_data.y * 1e7),  # longitude
        int(drone_data.z * 1000), # altitude
        int(drone_data.velocity_x * 100), # velocity X
        int(drone_data.velocity_y * 100), # velocity Y
        int(drone_data.velocity_z * 100), # velocity Z
        0 # heading, 예시에서는 무시
    )

# 데이터 수신 함수
def receive_data(mavlink_connection):
    while True:
        message = mavlink_connection.recv_match(blocking=True)
        if message is not None and message.get_type() == 'GLOBAL_POSITION_INT':
            drone_data = DroneData(
                message.sysid,
                message.lat * 1e-7,
                message.lon * 1e-7,
                message.alt * 1e-3,
                message.vx * 1e-2,
                message.vy * 1e-2,
                message.vz * 1e-2
            )
            # 여기에 수신된 데이터를 활용하는 로직 추가 가능

# 메인 함수
if __name__ == "__main__":
    connection = connect_mavlink("udp:localhost:14550")  # 예시 포트와 주소

    while True:
        # 현재 드론 데이터 수집 (이 부분은 실제 드론의 현재 상태를 가져오는 코드로 대체되어야 합니다.)
        current_data = DroneData(1, x, y, z, velocity_x, velocity_y, velocity_z)  # 예시 데이터

        # 데이터 전송
        send_data(connection, current_data)

        # 데이터 수신 및 처리
        receive_data(connection)

        # 일정 시간 간격으로 전송
        time.sleep(1)
