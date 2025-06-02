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
INIT_POSITION = 2048

# === 각도 → 위치 변환 ===
def angle_to_position_joint1(theta1_deg):
    return int(2048 - (theta1_deg - 180) * (1024 / 90))

def angle_to_position_joint2(theta2_deg):
    return int(2048 - theta2_deg * (2048 / 180))

# === 역기구학 ===
def inverse_kinematics(x, y):
    dist = np.sqrt(x**2 + y**2)
    if dist < abs(L1 - L2):
        raise ValueError("❌ 너무 가까워 도달 불가")
    if dist > (L1 + L2):
        raise ValueError("❌ 너무 멀어서 도달 불가")

    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)

    theta2_rad = np.arccos(D)
    theta2_deg = 180 - np.degrees(theta2_rad)

    # ✅ 좌표계 보정: θ1 = 0°일 때 정면(x+), 90°일 때 위쪽(y+)
    theta1_rad = np.arctan2(x, -y) - np.arctan2(
        L2 * np.sin(theta2_rad),
        L1 + L2 * np.cos(theta2_rad)
    )
    theta1_deg = np.degrees(theta1_rad)
    if theta1_deg < 0:
        theta1_deg += 360

    # 실제 동작 범위: [0°, 180°] (오른쪽 반원만)
    if not (0 <= theta1_deg <= 180):
        raise ValueError(f"❌ Joint1 범위 초과: θ1 = {theta1_deg:.2f}°")

    return theta1_deg, theta2_deg


# === 모터 초기화 ===
def motor_init():
    portHandler = PortHandler('COM12')  # 포트 확인
    packetHandler = PacketHandler(2.0)

    if not portHandler.openPort():
        raise RuntimeError("포트 열기 실패")
    if not portHandler.setBaudRate(57600):
        raise RuntimeError("보레이트 설정 실패")

    for motor_id in [0, 1]:
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_ACCELERATION, 30)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_VELOCITY, 100)
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, INIT_POSITION)

    return portHandler, packetHandler

# === 이동 명령 ===
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
        print(e)

# === 메인 루프 ===
if __name__ == "__main__":
    portHandler, packetHandler = motor_init()

    try:
        while True:
            user_input = input("목표 좌표 (x,y cm) 또는 'init': ").strip()
            if user_input.lower() == 'init':
                print("\n🔄 모터 초기화 중...\n")
                portHandler.closePort()
                portHandler, packetHandler = motor_init()
                continue

            try:
                x, y = map(float, user_input.split(','))
                move_to(x, y, portHandler, packetHandler)
            except:
                print("❗ 올바른 형식: x,y 또는 'init'")
    except KeyboardInterrupt:
        print("\n🛑 종료됨")
        portHandler.closePort()
