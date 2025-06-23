from dynamixel_sdk import *
import time

# 다이나믹셀 설정
DXL_ID = 4
BAUDRATE = 57600
DEVICENAME = 'COM3'  

# 주소 정의
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126

# 위치 및 파라미터
OPEN_POSITION = 3800
CLOSE_POSITION = 2500
CURRENT_THRESHOLD = 200     # 전류 임계값 (mA)
SLOW_SPEED = 50             # 속도
DELAY_MS = 0.05             # 반복 간격 (초)

# 16bit 값을 signed 값으로 변환하는 함수
def to_signed(val, bits):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)

# 포트 열기
if not portHandler.openPort():
    print("포트를 열 수 없습니다.")
    quit()

# 보드레이트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    quit()

# 모드 및 속도 설정
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 5)  # Current-based Position Mode
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PROFILE_VELOCITY, SLOW_SPEED)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

# 초기 상태: 그리퍼 열기
print(" 초기화: 그리퍼 열림 상태로 설정 중...")
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POSITION)
time.sleep(1.0)

# 사용자 입력 루프
try:
    while True:
        cmd = input("입력 ('open' 또는 'close', 'exit' 종료): ").strip().lower()

        if cmd == 'open':
            print(" 그리퍼 열기...")
            packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, OPEN_POSITION)

        elif cmd == 'close':
            print(" 그리퍼 닫기 시작... (전류 감지로 자동 멈춤)")
            packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, CLOSE_POSITION)

            while True:
                time.sleep(DELAY_MS)
                raw_current, _, _ = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
                current = to_signed(raw_current, 16)
                pos, _, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                print(f"전류값 : {current}")


                if abs(current) > CURRENT_THRESHOLD:
                    print(f" 물체 감지됨! 전류: {current} mA → 정지")
                    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pos)
                    break
                if pos < CLOSE_POSITION + 50:
                    break

        elif cmd == 'exit':
            print(" 프로그램 종료")
            break

        else:
            print(" 'open', 'close', 또는 'exit'만 입력 가능합니다.")

finally:
    portHandler.closePort()
