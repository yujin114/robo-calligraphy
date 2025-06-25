import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from wr_robot.multi_character_extractor_upgrade_base import MultiCharacterTrajectoryExtractor
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import time

class AllCharTrajectoryPublisher(Node):
    def __init__(self, sample_points=35):
        super().__init__('all_char_trajectory_publisher')
        self.sample_points = sample_points

        # QoS 설정
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # Trajectory 퍼블리셔
        self.publisher_ = self.create_publisher(Float32MultiArray, '/dsr01/all_chars_trajectory', qos)

        # font_style 구독자
        self.subscription_font_style = self.create_subscription(
            Int32,
            'font_style',  # GUI와 동일한 토픽 이름으로 변경
            self.cb_start_button,
            qos
        )

    def cb_start_button(self, msg):
        font_style = msg.data
        if font_style not in [0, 1]:
            self.get_logger().warn(f"🚫 잘못된 font_style 값: {font_style}")
            return

        time.sleep(3.0)  # GUI에서 이미지 저장 시간 고려

        # 이미지 경로 설정
        img_path = "/home/rokey/f3_ws/src/dr_writer/image/picture.png"

        # 경로 추출기 호출
        extractor = MultiCharacterTrajectoryExtractor(
            img_path=img_path,
            z_min=0.5,
            z_max=3.0,
            skeleton_mode='stroke'
        )
        char_dfs_resampled = extractor.get_all_dataframes_resampled(self.sample_points)

        # 데이터 합치기
        combined_data = []
        for char_idx, df in enumerate(char_dfs_resampled):
            xs = df['x'].to_numpy()
            ys = df['y'].to_numpy()
            zs = df['z'].to_numpy()

            total_len = len(xs)
            stride = max(1, total_len // self.sample_points)

            xs = xs[::stride][:self.sample_points]
            ys = ys[::stride][:self.sample_points]
            zs = zs[::stride][:self.sample_points]

            for x, y, z in zip(xs, ys, zs):
                combined_data.extend([float(x), float(y), float(z), float(char_idx)])

        # 메시지 퍼블리시
        msg_out = Float32MultiArray()
        msg_out.data = combined_data
        self.publisher_.publish(msg_out)
        self.get_logger().info(f'✅ font_style={font_style}, 총 {len(combined_data) // 4} 포인트 퍼블리시 완료')

def main():
    rclpy.init()
    node = AllCharTrajectoryPublisher(sample_points=35)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
