import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import defaultdict
import time

import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
FIXED_Z = -80.0
FIXED_RX = 41.76
FIXED_RY = -180.0
MAX_MOVESX_LENGTH = 127
FAST_ACC, FAST_VEL = 50, 50
SLOW_ACC, SLOW_VEL = 10, 10

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class MovePathNode(Node):
    def __init__(self):
        super().__init__('move_path', namespace=ROBOT_ID)
        self.pose_received = False
        self.path_list = []

        self.subscription = self.create_subscription(
            Float32MultiArray,
            f"/{ROBOT_ID}/all_chars_trajectory",
            self.listener_callback,
            10
        )
        self.get_logger().info(f"🟢 Subscription 등록됨: /{ROBOT_ID}/all_chars_trajectory")

    def listener_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) % 4 != 0:
            self.get_logger().error("❌ 길이가 4의 배수가 아닌 데이터를 받았습니다!")
            return

        stroke_dict = defaultdict(list)
        from DR_common2 import posx  # posx만 클래스 내부에서 잠깐 사용

        for i in range(0, len(data), 4):
            x, y, z, stroke_id = data[i], data[i + 1], data[i + 2], int(data[i + 3])
            # x += 300
            pose = posx(x, y, FIXED_Z, FIXED_RX, FIXED_RY, 0.0)
            stroke_dict[stroke_id].append(pose)

        self.path_list = [stroke_dict[sid] for sid in sorted(stroke_dict.keys())]
        self.pose_received = True

    def wait_for_pose(self):
        self.get_logger().info("📡 메시지 수신 대기 중...")
        while not self.pose_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        self.get_logger().info("✅ 메시지 수신 완료")


def main(args=None):
    rclpy.init(args=args)
    node = MovePathNode()
    DR_init.__dsr__node = node

    # ✅ Doosan API는 여기서 import
    from DSR_ROBOT2 import (
        movej,
        movel,
        movesx,
        get_tool,
        get_tcp,
        get_current_posx,set_user_cart_coord,set_ref_coord,
        DR_MVS_VEL_NONE,
        DR_BASE,
    )
    from DR_common2 import posj, posx

    # TCP/Tool 확인 ---------------------------------> REAL MODE 
    tool_name = get_tool()
    tcp_name = get_tcp()
    node.get_logger().info(f"🔧 Tool: {tool_name}, TCP: {tcp_name}")

    if tool_name == "" or tcp_name == "":
        node.get_logger().warn("❗ Tool 또는 TCP 정보가 설정되지 않았습니다. 종료합니다.")
        rclpy.shutdown()
        return
    # 사용자 좌표계 지정
    x_vector = [0,-1,0]
    y_vector = [-1,0,0]
    zero_point = posx([574.72 ,224.51 ,1.20 ,0 ,-180 ,0])

    DR_USER = set_user_cart_coord(x_vector, y_vector, zero_point)

    # 사용자 좌표계를 전역좌표계로 
    set_ref_coord(DR_USER)
    # 홈 위치로 이동
    home = posj(0, 0, 90.0, 0, 90, 0)
    movej(home, vel=100, acc=100)

    # 메시지 수신 대기
    node.wait_for_pose()

    total_moves = sum(len(stroke) for stroke in node.path_list)
    node.get_logger().info(f"✏️ 총 {total_moves}개의 경로 수신됨. stroke별 movesx 실행")

    for idx, stroke in enumerate(node.path_list):
        if not stroke:
            continue

        first_pose = stroke[0]
        node.get_logger().info(f"🌀 Stroke {idx} 시작: 총 {len(stroke)} 포인트")
        print(stroke)
        # ✅ 시작 위치 movel (z 고정)
        first_pose = list(first_pose)
        first_pose[2] = FIXED_Z - 60
        node.get_logger().info(f"🚀 Stroke {idx}: 첫 포인트로 movel 이동 (z={FIXED_Z})")
        movel(first_pose, vel=FAST_VEL, acc=FAST_ACC)

        # ✅ z 아래로 100 내려서 그리기 시작
        first_pose[2] += 30 
        movel(first_pose, vel=50, acc=50)

        # ✅ stroke 전체를 세그먼트 단위로 movesx
        for seg_idx in range(0, len(stroke), MAX_MOVESX_LENGTH):
            sub_path = stroke[seg_idx:seg_idx + MAX_MOVESX_LENGTH]
            node.get_logger().info(
                f"🛠️ Stroke {idx} - 세그먼트 {seg_idx // MAX_MOVESX_LENGTH + 1} 실행 ({len(sub_path)} 포인트)"
            )
        movesx(sub_path, vel=[30, 10], acc=[50, 10], vel_opt=DR_MVS_VEL_NONE)

        # ✅ 현재 위치 받아서 z만 보정
        # 1. get current position
        current = get_current_posx()
        
        # 2. 올바른 posx 형식인지 확인 후 z 수정하여 이동
        if current and len(current[0]) == 6:
            current = list(current[0])
            current[2] = FIXED_Z
            movel(current, vel=FAST_VEL, acc=FAST_ACC)
        else:
            node.get_logger().warn(f"❗ 잘못된 현재 위치 posx: {current}")

        time.sleep(0.1)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()