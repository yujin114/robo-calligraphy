import rclpy
import DR_init
import time
from std_msgs.msg import Float32MultiArray
from functools import partial

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, FIXED_ACC, DOWN_ACC = 10, 10, 10
FIXED_Z = 150.0
FIXED_RX = 41.76
FIXED_RY = -180.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 전역 변수
pose_received = False
path_list = []

# ⬇️ stroke index별로 경로를 분리하여 저장
from collections import defaultdict
MAX_MOVESX_LENGTH = 127

def listener_callback(msg, posx_func):
    global pose_received, path_list
    data = msg.data
    if len(data) % 4 != 0:
        print('❌ 길이가 4의 배수가 아닌 데이터를 받았습니다!')
        return

    stroke_dict = defaultdict(list)

    scale = 1.0
    origin_x = 200.0
    origin_y = -225.0

    for i in range(0, len(data), 4):
        x, y, z, stroke_id = data[i], data[i+1], data[i+2], int(data[i+3])
        cx = x * scale + origin_x
        cy = y * scale + origin_y
        cz = FIXED_Z

        pose = posx_func(cx, cy, cz, 180.0, 0.0, 0.0)  # ← 수정된 각도
        stroke_dict[stroke_id].append(pose)

    path_list.clear()
    for sid in sorted(stroke_dict.keys()):
        path_list.append(stroke_dict[sid])

    pose_received = True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_path", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej, movesx, movel, wait,
        set_digital_output, DR_BASE, DR_MVS_VEL_NONE,
    )
    from DR_common2 import posx, posj

    PEN_PORT = 1
    PEN_DOWN = 1
    PEN_UP = 0

    print("🟢 Subscription 등록 시도 중: /dsr01/all_chars_trajectory")

    node.create_subscription(
        Float32MultiArray,
        "/dsr01/all_chars_trajectory",
        partial(listener_callback, posx_func=posx),
        10
    )

    # 초기 자세로 이동
    home = posj(0, 0, 90.0, 0, 90, 0)
    movej(home, vel=VEL, acc=FIXED_ACC)

    node.get_logger().info("📡 메시지 수신 대기 중...")
    while not pose_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)

    total_moves = sum(len(stroke) for stroke in path_list)
    node.get_logger().info(f"✏️ 총 {total_moves}개의 경로 수신됨. stroke별 movesx 실행")

    for idx, stroke in enumerate(path_list):
        if not stroke:
            node.get_logger().warn(f"⚠️ Stroke {idx}는 비어있음")
            continue

        node.get_logger().info(f"🌀 Stroke {idx} 실행 시작 (총 {len(stroke)} 포인트)")

        # 첫 포인트로 이동 (펜 올린 상태)
        first_pose = stroke[0]
        node.get_logger().info(f"🚀 Stroke {idx}: 첫 포인트로 movel 이동 중...")
        node.get_logger().info(f"🔍 Stroke {idx} → first_pose = {first_pose}")
        set_digital_output(PEN_PORT, PEN_UP)
        movel(first_pose, vel=VEL, acc=FIXED_ACC)
        wait(0.5)  # 반드시 기다려야 함

        # 펜 내림
        set_digital_output(PEN_PORT, PEN_DOWN)
        time.sleep(0.2)

        # movesx 경로 따라 그리기
        for i in range(0, len(stroke), MAX_MOVESX_LENGTH):
            sub_path = stroke[i:i + MAX_MOVESX_LENGTH]
            node.get_logger().info(f"🛠️ Stroke {idx}: movesx 실행 - 세그먼트 {i // MAX_MOVESX_LENGTH + 1} ({len(sub_path)}개)")
            movesx(sub_path, vel=[30, 10], acc=[50, 10], vel_opt=DR_MVS_VEL_NONE)
            wait(0.5)

        # 펜 올림
        set_digital_output(PEN_PORT, PEN_UP)
        time.sleep(0.2)

        # 다음 stroke로 넘어가기 전에 약간 쉬기
        time.sleep(0.1)

    node.get_logger().info("✅ 모든 stroke 완료")


def main_entry():
    main()

if __name__ == "__main__":
    main()