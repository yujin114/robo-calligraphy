import matplotlib
matplotlib.use('TkAgg')  # 또는 headless 환경이면 'Agg'
import matplotlib.pyplot as plt
import numpy as np

# 반드시 같은 폴더 or PYTHONPATH에 있어야 함!
from multi_character_extractor_upgrade_base import MultiCharacterTrajectoryExtractor


def plot_skeleton_and_sampled_paths(orig_dataframes, sampled_dataframes, figsize=(10, 5)):
    """
    - orig_dataframes: 원본 경로 DataFrame 리스트 (resample 전)
    - sampled_dataframes: 샘플링 경로 DataFrame 리스트 (resample 후, n_points)
    """
    assert len(orig_dataframes) == len(sampled_dataframes)
    n_paths = len(orig_dataframes)
    cmap = plt.get_cmap('tab10', n_paths)
    fig, ax = plt.subplots(figsize=figsize)

    # y축 뒤집기 위한 범위 계산
    all_ys = np.concatenate([df['y'].to_numpy() for df in orig_dataframes])
    ymin, ymax = all_ys.min(), all_ys.max()
    margin = (ymax - ymin) * 0.2 + 10
    ax.set_aspect('equal')
    ax.set_ylim(ymax + margin, ymin - margin)
    
    for idx, (df_orig, df_samp) in enumerate(zip(orig_dataframes, sampled_dataframes)):
        xs_o = df_orig['x'].to_numpy()
        ys_o = df_orig['y'].to_numpy()
        xs_s = df_samp['x'].to_numpy()
        ys_s = df_samp['y'].to_numpy()
        zs_s = df_samp['z'].to_numpy()
        color = cmap(idx % cmap.N)
        
        # 1. 원본 path (연결 경로)
        ax.plot(xs_o, ys_o, linestyle='dashed', color='gray', linewidth=1, label=f'orig_{idx}' if idx==0 else None)
        ax.scatter(xs_o, ys_o, color='gray', s=8, alpha=0.7)

        # 2. 샘플링 path
        ax.plot(xs_s, ys_s, color=color, linewidth=2, alpha=0.6, label=f'sampled_{idx}' if idx==0 else None)
        ax.scatter(xs_s, ys_s, c=zs_s, cmap='jet', s=30, edgecolor='black', zorder=10)

        # 샘플링 좌표에 인덱스 표시
        for i, (x, y) in enumerate(zip(xs_s, ys_s)):
            ax.text(x, y, str(i), fontsize=7, color='blue', ha='center', va='center')

        # path 시작/끝점 마커
        ax.plot(xs_s[0], ys_s[0], marker='o', color='black', markersize=8)
        ax.plot(xs_s[-1], ys_s[-1], marker='s', color='red', markersize=8)

    ax.set_xlabel('x (px)')
    ax.set_ylabel('y (px)')
    ax.set_title('Original vs. Sampled Path')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # 이미지 경로와 샘플링 개수 지정
    img_path = '/home/yujin/f6_ws/src/wr_robot/image/picture.png'      # 실제 이미지 경로로 변경
    n_points = 20               # 원하는 샘플링 개수

    # extractor 객체 생성 (dr_writer.multi_character_extractor_upgrade_base에 정의되어 있어야 함)
    extractor = MultiCharacterTrajectoryExtractor(
        img_path=img_path,
        z_min=0.5,
        z_max=3.0,
        skeleton_mode='stroke'
    )

    # extractor = MultiCharacterTrajectoryExtractor(...)
    orig_dfs = extractor.get_all_dataframes()                # 원본
    samp_dfs = extractor.get_all_dataframes_resampled(20)    # 샘플링 후 (ex: 20개)
    plot_skeleton_and_sampled_paths(orig_dfs, samp_dfs)
