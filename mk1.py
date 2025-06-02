import numpy as np
from dynamixel_sdk import *

# === 링크 길이 (단위: cm) ===
L1 = 20.0
L2 = 16.0


# === 다이나믹셀 주소값 ===
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_ACCELERATION = 108
ADDR_VELOCITY = 112

TORQUE_ENABLE = 1
INIT_POSITION = 2048  # 기준 위치 (중앙)

# === Joint1: θ1 [-90°, 90°] → 3072 ~ 1024 ===
def angle_to_position_joint1(theta1_deg):
    return int(2048 - theta1_deg * (1024 / 90))

# === Joint2: θ2 [-135°, 135°] → 523 ~ 3573 ===
def angle_to_position_joint2(theta2_deg):
    # Joint2의 θ2는 -135°(펼침) ~ +135°(접힘)
    # 위치값은 523 ~ 3573 사이
    return int(2048 + theta2_deg * (3050 / 270))


# === 역기구학 계산: (x, y) → θ1, θ2 ===
def inverse_kinematics(x, y):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        raise ValueError("도달할 수 없는 위치입니다.")
    
    # Link2 기준 각도
    theta2_rad = np.arccos(D)
    theta1_rad = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2_rad), L1 + L2*np.cos(theta2_rad))

    # 각도 보정
    theta1_deg = np.degrees(theta1_rad)
    theta2_deg = 180 - np.degrees(theta2_rad)  # θ2 = 180°일 때 완전 펼침

    # 제한 범위 확인
    if not (-90 <= theta1_deg <= 90):
        raise ValueError("❌ Joint1 범위 초과: θ1 = {:.2f}°".format(theta1_deg))
    if not (-135 <= theta2_deg <= 135):
        raise ValueError("❌ Joint2 범위 초과: θ2 = {:.2f}°".format(theta2_deg))

    return theta1_deg, theta2_deg

# === 모터 초기화 및 기준 위치로 이동 ===
def motor_init():
    portHandler = PortHandler('COM12')  # 너의 포트 번호로 수정!
    packetHandler = PacketHandler(2.0)

    if not portHandler.openPort():
        raise RuntimeError("포트 열기 실패")
    if not portHandler.setBaudRate(57600):
        raise RuntimeError("보레이트 설정 실패")

    for motor_id in [0, 1]:
        # 속도/가속도 설정
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_ACCELERATION, 30)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_VELOCITY, 100)

        # 토크 ON
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        # 기준 위치로 초기화
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, INIT_POSITION)

    return portHandler, packetHandler

# === 이동 명령 함수 ===
def move_to(x, y, portHandler, packetHandler):
    try:
        theta1, theta2 = inverse_kinematics(x, y)
        pos1 = angle_to_position_joint1(theta1)
        pos2 = angle_to_position_joint2(theta2)

        packetHandler.write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, pos1)
        packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, pos2)

        print(f"\n✅ 입력 좌표: ({x:.1f}, {y:.1f})")
        print(f"   → θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°")
        print(f"   → pos1 = {pos1}, pos2 = {pos2}\n")
    except ValueError as e:
        print(str(e))

# === 메인 루프 ===
if __name__ == "__main__":
    portHandler, packetHandler = motor_init()

    try:
        while True:
            user_input = input("목표 좌표 (x,y cm) 또는 'init': ").strip()
            if user_input.lower() == 'init':
                print("\n 모터 초기화 중...\n")
                portHandler.closePort()
                portHandler, packetHandler = motor_init()
                continue

            try:
                x, y = map(float, user_input.split(','))
                move_to(x, y, portHandler, packetHandler)
            except:
                print(" 올바른 형식: x,y 또는 'init'")
    except KeyboardInterrupt:
        print("\n 종료됨")
        portHandler.closePort()
