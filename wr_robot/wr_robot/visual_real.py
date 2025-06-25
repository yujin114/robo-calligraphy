import matplotlib
matplotlib.use('TkAgg')
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dr_writer.multi_character_extractor_upgrade_base import MultiCharacterTrajectoryExtractor
from ament_index_python.packages import get_package_share_directory

import numpy as np
import matplotlib.pyplot as plt
import os


class AllCharTrajectoryPublisher(Node):
    def __init__(self, img_path, sample_points=20):
        super().__init__('visual')
        self.sample_points = sample_points
        self.publisher_ = self.create_publisher(Float32MultiArray, '/dsr01/all_chars_trajectory', 10)

        # 1. Trajectory 추출
        extractor = MultiCharacterTrajectoryExtractor(
            img_path=img_path,
            z_min=0.5,
            z_max=3.0,
            skeleton_mode='stroke'
        )
        all_dataframes = extractor.get_all_dataframes()

        # 2. 모든 char 데이터 통합
        self.combined_data = []
        self.debug_points = []

        for char_idx, df in enumerate(all_dataframes):
            xs = df['x'].to_numpy()
            ys = df['y'].to_numpy()
            zs = df['z'].to_numpy()
            total_len = len(xs)

            if total_len > sample_points:
                stride = max(1, total_len // sample_points)
                xs = xs[::stride][:sample_points]
                ys = ys[::stride][:sample_points]
                zs = zs[::stride][:sample_points]

            for x, y, z in zip(xs, ys, zs):
                self.combined_data.extend([float(x), float(y), float(z), float(char_idx)])
                self.debug_points.append((x, y, z, char_idx))

        # 3. 메시지 생성 및 퍼블리시
        msg = Float32MultiArray()
        msg.data = self.combined_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'✅ Published total {len(self.combined_data) // 4} points to /all_chars_trajectory')


def visualize_data(data):
    xs, ys, zs, char_ids = [], [], [], []
    for i in range(0, len(data), 4):
        xs.append(data[i])
        ys.append(data[i + 1])
        zs.append(data[i + 2])
        char_ids.append(int(data[i + 3]))

    # 시각화 (문자별 색상, 순서 텍스트 포함)
    unique_chars = sorted(set(char_ids))
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i % 10) for i in char_ids]

    plt.figure(figsize=(8, 8))
    for i in range(len(xs)):
        plt.scatter(xs[i], ys[i], c=[colors[i]], s=30)
        plt.text(xs[i], ys[i], str(i), fontsize=8, ha='center', va='center')  # 포인트 순서 라벨

    plt.title('Character Stroke Trajectories (2D with Order)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().invert_yaxis()  # 이미지 좌표계 보정
    plt.axis('equal')
    plt.grid(True)
    plt.show()


def main():
    rclpy.init()

    # 이미지 경로 지정
    pkg_share = get_package_share_directory('dr_writer')
    img_path = os.path.join(pkg_share, 'pictures', 'r3.png')

    # 노드 생성 및 데이터 추출
    node = AllCharTrajectoryPublisher(img_path=img_path, sample_points=20)
    rclpy.spin_once(node, timeout_sec=0.1)

    # 시각화 수행
    visualize_data(node.combined_data)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
