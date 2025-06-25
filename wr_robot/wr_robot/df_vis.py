import matplotlib
matplotlib.use('TkAgg')  # 또는 headless 환경이면 'Agg'
import matplotlib.pyplot as plt
import numpy as np

# 반드시 같은 폴더 or PYTHONPATH에 있어야 함!
from multi_character_extractor_upgrade_base import MultiCharacterTrajectoryExtractor

def plot_trajectories(dataframes, show_z_as_color=True, figsize=(10, 5), with_markers=True):
    n_paths = len(dataframes)
    char_idxs = sorted(set(df['char_idx'].iloc[0] for df in dataframes))
    cmap = plt.get_cmap('tab10', len(char_idxs))
    fig, ax = plt.subplots(figsize=figsize)

    all_ys = np.concatenate([df['y'].to_numpy() for df in dataframes])
    ymin, ymax = all_ys.min(), all_ys.max()
    margin = (ymax - ymin) * 0.2 + 10
    ax.set_aspect('equal')
    ax.set_ylim(ymax + margin, ymin - margin)

    for idx, df in enumerate(dataframes):
        xs = df['x'].to_numpy()
        ys = df['y'].to_numpy()
        zs = df['z'].to_numpy()
        char_idx = df['char_idx'].iloc[0]
        color = cmap(char_idx % cmap.N)
        # trajectory plot
        if show_z_as_color:
            # sc = ax.scatter(xs, ys, c=zs, cmap='jet', label=f'char{char_idx}_p{idx}', s=15)
            ax.plot(xs, ys, color=color, alpha=0.4)
        else:
            ax.plot(xs, ys, color=color, label=f'char{char_idx}_p{idx}', lw=2)
        # 각 점에 인덱스 표시 (여기 추가)
        for i, (x, y) in enumerate(zip(xs, ys)):
            ax.text(x, y, str(i), fontsize=7, color='blue', ha='center', va='center')
        # path 시작/끝점 마커
        if with_markers:
            ax.plot(xs[0], ys[0], marker='o', color='black', markersize=3)
            ax.plot(xs[-1], ys[-1], marker='s', color='red', markersize=3)
            ax.text(xs[0], ys[0]-5, f'{idx}', fontsize=9, color='black')

    ax.set_xlabel('x (px)')
    ax.set_ylabel('y (px)')
    ax.set_title('char_idx')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # 이미지 경로와 샘플링 개수 지정
    img_path = '/home/yujin/f6_ws/src/wr_robot/image/picture.png'      # 실제 이미지 경로로 변경
    n_points = 30               # 원하는 샘플링 개수

    # extractor 객체 생성 (dr_writer.multi_character_extractor_upgrade_base에 정의되어 있어야 함)
    extractor = MultiCharacterTrajectoryExtractor(
        img_path=img_path,
        z_min=0.5,
        z_max=3.0,
        skeleton_mode='stroke'
    )

    # 모든 path마다 등간격 n_points 샘플링
    char_dfs_resampled = extractor.get_all_dataframes_resampled(n_points)
    print(char_dfs_resampled)

    # 시각화 함수 호출
    plot_trajectories(char_dfs_resampled, show_z_as_color=True)
