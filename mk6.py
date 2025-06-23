import numpy as np
import time
from dynamixel_sdk import *

# === 링크 길이 (단위: cm) ===
L1 = 20.0
L2 = 16.0

# === 다이나믹셀 주소값 ===
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_ACCELERATION = 108
ADDR_VELOCITY = 112
ADDR_PROFILE_VELOCITY = 112
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126

# === 파라미터 ===
TORQUE_ENABLE = 1
INIT_POSITION = 2048
SLOW_SPEED = 50
OPEN_POSITION = 3800
CLOSE_POSITION = 2500
CURRENT_THRESHOLD = 200
DELAY_MS = 0.05

# === 각도 → 위치 변환 함수들 ===
def angle_to_position_joint1(theta1_deg):
    return int(2048 - theta1_deg * (1024 / 90))

def angle_to_position_joint2(theta2_deg):
    if theta2_deg < 180:
        return int(2048 - theta2_deg * (2048 / 180))
    else:
        theta2_deg -= 180
        return int(2048 + theta2_deg * (2048 / 180))

def angle_to_position_joint3(theta3_deg):
    return int(2048 + theta3_deg * (1024 / 90))  # -90° ~ 90°

# 16bit signed 변환
def to_signed(val, bits):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

# === 역기구학 ===
def inverse_kinematics(x, y):
    x_2 = x
    x = abs(x)
    y = abs(y)
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1:
        raise ValueError("도달불가")
    theta2_rad = np.arccos(D)
    theta2_deg = 180 - np.degrees(theta2_rad)
    theta1_rad = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2_rad), L1 + L2*np.cos(theta2_rad))
    theta1_deg = np.degrees(theta1_rad)
    theta3_deg = theta2_deg - theta1_deg

    if x_2 < 0:
        theta1_deg *= -1
        theta2_deg = 180 + theta2_deg
        theta3_deg = 180 - theta3_deg
        
    if not (-90 <= theta1_deg <= 90):
        raise ValueError(f"Joint1 범위 초과: θ1 = {theta1_deg:.2f}°")
    return theta1_deg, theta2_deg, theta3_deg 

# === 모터 초기화 ===
def motor_init():
    portHandler = PortHandler('COM3')
    packetHandler = PacketHandler(2.0)
    if not portHandler.openPort():
        raise RuntimeError("포트 열기 실패")
    if not portHandler.setBaudRate(57600):
        raise RuntimeError("보레이트 설정 실패")
    
    for motor_id in [0, 1, 3]:  # Joint1, Joint2, Joint3
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_ACCELERATION, 30)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_VELOCITY, 100)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, INIT_POSITION)

    # === 그리퍼 모터 초기화 ===
    packetHandler.write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, 4, ADDR_OPERATING_MODE, 5)  # Current-based Position Mode
    packetHandler.write4ByteTxRx(portHandler, 4, ADDR_PROFILE_VELOCITY, SLOW_SPEED)
    packetHandler.write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION)
    time.sleep(1.0)

    return portHandler, packetHandler

# === 이동 명령 ===
def move_to(x, y, portHandler, packetHandler):
    try:
        theta1, theta2, theta3 = inverse_kinematics(x, y)
        # theta3 = theta2 - theta1  # 그리퍼 회전 보정

        pos1 = angle_to_position_joint1(theta1)
        pos2 = angle_to_position_joint2(theta2)
        pos3 = angle_to_position_joint3(theta3)

        packetHandler.write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, pos1)
        packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, pos2)
        packetHandler.write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, pos3)

        print(f"\n 입력 좌표: ({x:.1f}, {y:.1f})")
        print(f"   → θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°, θ3 (Gripper 회전) = {theta3:.2f}°")
        print(f"   → pos1 = {pos1}, pos2 = {pos2}, pos3 = {pos3}\n")
    except ValueError as e:
        print(str(e))


# === 그리퍼 열기 ===
def open_gripper(portHandler, packetHandler):
    print(" 그리퍼 열기...")
    packetHandler.write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION)

# === 그리퍼 닫기 ===
def close_gripper(portHandler, packetHandler):
    print(" 그리퍼 닫기... (전류 감지 중)")
    packetHandler.write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, CLOSE_POSITION)
    while True:
        time.sleep(DELAY_MS)
        raw_current, _, _ = packetHandler.read2ByteTxRx(portHandler, 4, ADDR_PRESENT_CURRENT)
        current = to_signed(raw_current, 16)
        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, 4, ADDR_PRESENT_POSITION)
        print(f"전류값 : {current}")
        if abs(current) > CURRENT_THRESHOLD:
            print(f" 물체 감지됨! 전류: {current} mA → 정지")
            packetHandler.write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, pos)
            break
        if pos < CLOSE_POSITION + 50:
            break

# === 메인 루프 ===
if __name__ == "__main__":
    portHandler, packetHandler = motor_init()

    try:
        while True:
            cmd = input("\n명령 입력 (x,y / open / close / init / exit): ").strip().lower()

            if cmd == 'init':
                print(" 모터 재초기화 중...")
                portHandler.closePort()
                portHandler, packetHandler = motor_init()

            elif cmd == 'open':
                open_gripper(portHandler, packetHandler)

            elif cmd == 'close':
                close_gripper(portHandler, packetHandler)

            elif cmd == 'exit':
                print("종료")
                break

            else:
                try:
                    x, y = map(float, cmd.split(','))
                    move_to(x, y, portHandler, packetHandler)
                except:
                    print("올바른 입력이 아닙니다. (예: 10,5 / open / close / init / exit)")

    except KeyboardInterrupt:
        print("\n종료됨")
    finally:
        portHandler.closePort()
