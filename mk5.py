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

# === 각도 → 위치 변환 함수들 ===

# Joint1
def angle_to_position_joint1(theta1_deg):
    return int(2048 - theta1_deg * (1024 / 90))

# Joint2
def angle_to_position_joint2(theta2_deg):
    if theta2_deg < 180:
        return int(2048 - theta2_deg * (2048 / 180))
    else:
        theta2_deg -= 180
        return int(2048 + theta2_deg * (2048 / 180))

# Joint3 (Gripper 회전)
def angle_to_position_joint3(theta3_deg):
    return int(2048 + theta3_deg * (1024 / 90))  # -90° ~ 90° 기준

# === 역기구학 ===
def inverse_kinematics(x, y):
    x_2 = x
    x = abs(x)
    y = abs(y)

    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        raise ValueError("도달불가")

    theta2_rad = np.arccos(D)
    theta2_deg = np.degrees(theta2_rad)
    theta2_deg = 180 - theta2_deg

    theta1_rad = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2_rad), L1 + L2*np.cos(theta2_rad))
    theta1_deg = np.degrees(theta1_rad)

    # 대칭 처리
    if x_2 < 0:
        theta1_deg *= -1
        theta2_deg = 180 + theta2_deg  # 180 기준 대칭

    if not (-90 <= theta1_deg <= 90):
        raise ValueError(f"Joint1 범위 초과: θ1 = {theta1_deg:.2f}°")

    return theta1_deg, theta2_deg

# === 모터 초기화 ===
def motor_init():
    portHandler = PortHandler('COM3')  
    packetHandler = PacketHandler(2.0)

    if not portHandler.openPort():
        raise RuntimeError("포트 열기 실패")
    if not portHandler.setBaudRate(57600):
        raise RuntimeError("보레이트 설정 실패")

    for motor_id in [0, 1, 3]:  # Joint1, Joint2, Joint3(Gripper)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_ACCELERATION, 30)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_VELOCITY, 100)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, INIT_POSITION)

    return portHandler, packetHandler

# === 이동 명령 ===
def move_to(x, y, portHandler, packetHandler):
    try:
        theta1, theta2 = inverse_kinematics(x, y)
        theta3 = theta2 - theta1  # 그리퍼 회전 각도 보정

        # 각도 → 위치값 변환
        pos1 = angle_to_position_joint1(theta1)
        pos2 = angle_to_position_joint2(theta2)
        pos3 = angle_to_position_joint3(theta3)

        # 각 모터에 명령 전송
        packetHandler.write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, pos1)
        packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, pos2)
        packetHandler.write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, pos3)

        # 결과 출력
        print(f"\n 입력 좌표: ({x:.1f}, {y:.1f})")
        print(f"   → θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°, θ3 = {theta3:.2f}°")
        print(f"   → pos1 = {pos1}, pos2 = {pos2}, pos3 = {pos3}\n")

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
        print("\n종료됨")
        portHandler.closePort()
