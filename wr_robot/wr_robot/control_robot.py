import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, Bool
from collections import defaultdict
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import DR_init



# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
FIXED_Z = 75.0
FIXED_RX = 41.76
FIXED_RY = -180.0
MAX_MOVESX_LENGTH = 127
FAST_ACC, FAST_VEL = 50, 50
SLOW_ACC, SLOW_VEL = 10, 10
ON , OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def convert_x(x):
    return 400 - x

class MovePathNode(Node):
    def __init__(self):
        super().__init__('move_path', namespace=ROBOT_ID)
        self.pose_received = False
        self.path_list = []
        self.thickness = 0
        self.active = False
        self.shutdown_request = False

        self.subscription = self.create_subscription(
            Float32MultiArray,
            f"/{ROBOT_ID}/all_chars_trajectory",
            self.listener_callback,
            10
        )

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.subscription_ui = self.create_subscription(
                Int32,
                "/font_style",
                self.cb_ui,
                qos
        )

        self.shutdown_sub = self.create_subscription(
            Bool,
            '/shutdown_signal',
            self.shutdown_callback,
            qos
        )

        self.get_logger().info(f"🟢 Subscription 등록됨: /{ROBOT_ID}/all_chars_trajectory")
        self.get_logger().info(f"🟢 Subscription 등록됨: /font_style")

    def cb_ui(self,msg : Int32):
        data = msg.data
        if data == 0:
            self.thickness = 73.5
        elif data ==1:
            self.thickness = 71

    def listener_callback(self, msg: Float32MultiArray):
        if self.active:
            self.get_logger().warn("글씨 쓰는 중... 잠시 후 재시도 하시오")
            return
        self.active = True
        
        data = msg.data
        if len(data) % 4 != 0:
            self.get_logger().error("❌ 길이가 4의 배수가 아닌 데이터를 받았습니다!")
            return

        # self.thickness 기다리기
        while self.thickness == 0 and rclpy.ok():
            self.get_logger().warn("⏳ thickness 값 대기 중...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)


        FIXED_Z = self.thickness
        self.get_logger().info(f"✏️ z_높이 =  {FIXED_Z}")

        stroke_dict = defaultdict(list)
        from DR_common2 import posx  # posx만 클래스 내부에서 잠깐 사용

        for i in range(0, len(data), 4):
            x_, y, z, stroke_id = data[i], data[i + 1], data[i + 2], int(data[i + 3])
            x = convert_x(x_)
            x += 300
            
            pose = posx(x, y, FIXED_Z, FIXED_RX, FIXED_RY, 0.0)
            stroke_dict[stroke_id].append(pose)

        self.path_list = [stroke_dict[sid] for sid in sorted(stroke_dict.keys())]
        self.pose_received = True
        print(f'fixed_Z :{self.thickness}')

    def wait_for_pose(self):
        self.get_logger().info("📡 메시지 수신 대기 중...")
        while not self.pose_received and rclpy.ok(): 
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        self.get_logger().info("✅ 메시지 수신 완료")


    
    def shutdown_callback(self, msg: Bool):
        
        signal = msg.data
        if signal:
            self.shutdown_request = True
            self.pose_received = True
        
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
        get_current_posx,
        set_digital_output,
        wait,
        set_tool, set_tcp,
        DR_MVS_VEL_CONST,
        DR_BASE,
    )
    from DR_common2 import posj, posx

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")


    # TCP/Tool 확인 ---------------------------------> REAL MODE 
    tool_name = get_tool()
    tcp_name = get_tcp()
    node.get_logger().info(f"🔧 Tool: {tool_name}, TCP: {tcp_name}")

    if tool_name == "" or tcp_name == "":
        node.get_logger().warn("❗ Tool 또는 TCP 정보가 설정되지 않았습니다. 종료합니다.")
        rclpy.shutdown()
        return

    # 홈 위치로 이동
    home = posj(0, 0, 90.0, 0, 90, 0)

    movej(home, vel=80, acc=80)
    pos_start_pen_down = posx([483.53, 258.11 , 111.32, 13.97, -179.91, 14.27])
    pos_start_pen_up = posx([483.53, 258.11 , 211.37, 13.97, -179.91, 14.27])

    movel(pos_start_pen_up, vel = 100, acc= 50)
    movel(pos_start_pen_down, vel = 70, acc= 50)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1.0)
    movel(pos_start_pen_up, vel = 70, acc= 50)
    movej(home, vel=100, acc=100)


    while rclpy.ok():
        try:
            # 메시지 수신 대기
            node.wait_for_pose()
            if node.shutdown_request:
                node.get_logger().info("❌ shutdown 요청 수신됨! 종료합니다.")
                pos_start_pen_down = posx([483.53, 258.11 , 111.32, 13.97, -179.91, 14.27])
                pos_start_pen_up = posx([483.53, 258.11 , 211.37, 13.97, -179.91, 14.27])
                home = posj(0, 0, 90.0, 0, 90, 0)
                movej(home, vel=100, acc=100)
                movel(pos_start_pen_up, vel = 100, acc= 50)
                movel(pos_start_pen_down, vel = 70, acc= 50)
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                wait(1.0)
                movel(pos_start_pen_up, vel = 70, acc= 50)
                movej(home, vel=100, acc=100)
                time.sleep(5.0)
                rclpy.shutdown()
            else:
                pass

            total_moves = sum(len(stroke) for stroke in node.path_list)
            FIXED_Z = node.thickness
            node.get_logger().info(f"✏️ z_높이 =  {FIXED_Z}")
            node.get_logger().info(f"✏️ 총 {total_moves}개의 경로 수신됨. stroke별 movesx 실행")
            node.get_logger().info(f"✏️ 글씨 굵기 {'굵게' if FIXED_Z == 75 else '얇게'}")


            for idx, stroke in enumerate(node.path_list):
                if not stroke:
                    continue

                first_pose = stroke[0]
                node.get_logger().info(f"🌀 Stroke {idx} 시작: 총 {len(stroke)} 포인트")
                print(stroke)
                # ✅ 시작 위치 movel (z 고정)
                first_pose = list(first_pose)
                first_pose[2] = FIXED_Z + 80
                node.get_logger().info(f"🚀 Stroke {idx}: 첫 포인트로 movel 이동 (z={FIXED_Z})")
                movel(first_pose, vel=FAST_VEL, acc=FAST_ACC, ref=DR_BASE)

                # ✅ z 아래로 80 내려서 그리기 시작
                first_pose[2] -= 80 
                movel(first_pose, vel=50, acc=50)

                # ✅ stroke 전체를 세그먼트 단위로 movesx
                # for seg_idx in range(0, len(stroke), MAX_MOVESX_LENGTH):
                #     sub_path = stroke[seg_idx:seg_idx + MAX_MOVESX_LENGTH]
                #     node.get_logger().info(
                #         f"🛠️ Stroke {idx} - 세그먼트 {seg_idx // MAX_MOVESX_LENGTH + 1} 실행 ({len(sub_path)} 포인트)"
                #     )
                movesx(stroke, vel=[10,5 ], acc=[10, 5], vel_opt=DR_MVS_VEL_CONST)

                # ✅ 현재 위치 받아서 z만 보정
                # 1. get current position
                current = get_current_posx()
                
                # 2. 올바른 posx 형식인지 확인 후 z 수정하여 이동
                if current and len(current[0]) == 6:
                    current = list(current[0])
                    current[2] = FIXED_Z + 80
                    movel(current, vel=FAST_VEL, acc=FAST_ACC, ref=DR_BASE)
                else:
                    node.get_logger().warn(f"❗ 잘못된 현재 위치 posx: {current}")

                time.sleep(0.1)
            movej(home, vel=80, acc=80)
        except Exception as e:
            node.get_logger().error(f"❌ 예외 발생: {e}")
        finally:
            node.active = False
            node.pose_received = False
            node.get_logger().info("💤 다음 입력을 기다립니다...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()