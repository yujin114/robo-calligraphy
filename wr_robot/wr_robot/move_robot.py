import rclpy
import DR_init
import time
import math
import queue
from std_msgs.msg import Float32MultiArray

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, FIXED_ACC, DOWN_ACC = 100, 100, 30
FIXED_Z = 150.0
DOWN_Z = 150.0
FIXED_RX = 41.76
FIXED_RY = -180.0
ON, OFF = 1, 0
Z_PEN_THRESHOLD = 1.5

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

strokes_queue = queue.Queue()
pose_received = False

def image_to_robot_coords(x_image, y_image):
    x_robot = int(x_image * (400/77))
    y_robot = int(y_image * (400/77))
    return x_robot, y_robot

def listener_callback(msg):
    global pose_received
    data = msg.data
    if len(data) % 4 != 0:
        print('길이가 4의 배수가 아닌 데이터를 받았습니다!')
        return
    strokes = [[data[i], data[i+1], data[i+2], int(data[i+3])] for i in range(0, len(data), 4)]
    strokes_queue.put(strokes)
    pose_received = True

def convert_x(x):
    return 400 - x

def convert_y(y):
    return y  # y는 그대로

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movel,
        movej,
        movesx,
        wait,
        set_digital_output,
        get_tool,
        get_tcp,
        DR_TOOL,
        task_compliance_ctrl,
        set_desired_force,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        check_force_condition,
        DR_BASE,
        release_force,
        release_compliance_ctrl,
    )
    from DR_common2 import posx, posj

    print("🟢 Subscription 등록 시도 중: /dsr01/all_chars_trajectory")
    node.create_subscription(Float32MultiArray, "/dsr01/all_chars_trajectory", listener_callback, 10)

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)
        release_compliance_ctrl()
        release_force()

    def pen_down():
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force([0, 0, 20, 0, 0, 0], [0, 0, 1, 0, 0, 0], DR_FC_MOD_REL)

    def pen_up():
        release()
        time.sleep(0.1)

    def draw_path(points):
        last_char_idx = None

        for i, (xc, yc, z, char_idx) in enumerate(points):
            x, y = image_to_robot_coords(xc, yc)
            target = posx(x, y, FIXED_Z, FIXED_RX, FIXED_RY, 0)

            if i == 0 or char_idx != last_char_idx:
                movel(target, VEL, FIXED_ACC, ref=DR_BASE)
                pen_down()
            else:
                movesx([prev_target, target], VEL, FIXED_ACC)

            prev_target = target
            last_char_idx = char_idx

        pen_up()

    home = posj(0, 0, 90.0, 0, 90, 0)
    movej(home, vel=VEL, acc=FIXED_ACC)

    node.get_logger().info("📡 메시지 수신 대기 중...")
    while not pose_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)

    node.get_logger().info("✏️ 메시지 수신됨, 경로 그리기 시작")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not strokes_queue.empty():
                data = strokes_queue.get()
                draw_path(data)

    except KeyboardInterrupt:
        release()
        node.get_logger().info('Shutting down...')

    node.destroy_node()
    rclpy.shutdown()

def main_entry():
    main()

if __name__ == "__main__":
    main()