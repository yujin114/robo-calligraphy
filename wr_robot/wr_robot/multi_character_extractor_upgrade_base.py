import cv2
import numpy as np
from skimage.morphology import skeletonize, medial_axis
import pandas as pd
from itertools import chain

from skeleton_dfs_base import extract_all_paths  # DFS 기반 중심선 경로 추출 함수 (외부 구현)

def filter_nearby_points(path, min_dist=2.0):
    """
    경로 상 너무 가까운 점 제거
    최소 거리 이상 떨어진 포인트만 남김
    입력 path: [[x1, y1], [x2, y2], ...]
    """
    if not path:
        return path
    filtered = [path[0]]
    for pt in path[1:]:
        prev = filtered[-1]
        if np.hypot(pt[0] - prev[0], pt[1] - prev[1]) >= min_dist:
            filtered.append(pt)
    return filtered

def flip_path_to_start_near_origin(path):
    """
    path의 양 끝점 중 x좌표가 0에 더 가까운 쪽이 path[0]이 되도록,
    아니면 path를 뒤집어서 시작점을 맞춘다.
    """
    if not path or len(path) < 2:
        return path
    x0 = abs(path[0][0])   # path[0][0] = x좌표
    x1 = abs(path[-1][0])  # path[-1][0] = x좌표
    if x0 <= x1:
        return path
    else:
        return path[::-1]

def binarize_image(img):
    """
    이미지를 이진화합니다.
    Otsu 알고리즘을 이용해 임계값을 자동 계산, 흑백으로 변환합니다.
    픽셀 값이 0 또는 1로 나오도록 설정 (OpenCV는 기본 0/255지만, 여기서는 0/1로 변환)
    """
    _, binary = cv2.threshold(img, 0, 1, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return binary

def extract_skeleton(mask, mode='skeletonize'):
    """
    입력 마스크(이진 이미지)에서 중심선/윤곽선 등 Skeleton 추출  
    mode에 따라 다양한 방식 선택 가능:
      - 'skeletonize': 일반적인 skeletonize 방식 (skimage)
      - 'medial': medial_axis로 중심선 추출 (skimage)
      - 'outer': 외곽선 두껍게 그린 뒤 mask화
      - 'stroke': 외곽선 따라 두껍게 그리고 skeletonize 처리
    """
    if mode == 'skeletonize':
        return skeletonize(mask > 0).astype(np.uint8)
    elif mode == 'medial':
        medial, _ = medial_axis(mask > 0, return_distance=True)
        return medial.astype(np.uint8)
    elif mode == 'outer':
        contours, hierarchy = cv2.findContours(mask.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return np.zeros_like(mask)
        skeleton_like = np.zeros_like(mask)
        hierarchy = hierarchy[0]
        for i, cnt in enumerate(contours):
            if hierarchy[i][3] == -1:
                cv2.drawContours(skeleton_like, [cnt], -1, 1, thickness=7)
        return skeleton_like
    elif mode == 'stroke':
        filled = np.zeros_like(mask)
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(filled, contours, -1, 1, thickness=3)
        skeleton = skeletonize(filled > 0).astype(np.uint8)
        return skeleton
    else:
        raise ValueError(f"Unknown skeleton mode: {mode}")

def get_local_thickness(y, x, binary, max_search=30):
    """
    (x, y) 좌표 기준 네 방향(상/하/좌/우)으로 몇 칸씩 흰색(1) 픽셀이 이어지는지 검사  
    각 방향마다 연결된 픽셀 수를 세어, 총합을 local thickness로 사용
    """
    H, W = binary.shape
    dists = []
    for dy, dx in [(-1,0), (1,0), (0,-1), (0,1)]:
        cnt = 0
        ny, nx = y+dy, x+dx
        while 0 <= ny < H and 0 <= nx < W and binary[ny, nx]:
            cnt += 1
            ny += dy
            nx += dx
            if cnt > max_search:
                break
        dists.append(cnt)
    return sum(dists)

def normalize_thickness(thicknesses, z_min, z_max):
    """
    thickness 리스트를 받아, z_min~z_max 범위로 정규화  
    만약 두께값이 전부 같으면(z.ptp()==0), 전부 z_min으로 고정
    """
    thicknesses = np.array(thicknesses)
    if thicknesses.ptp() == 0:
        return np.full_like(thicknesses, z_min, dtype=float)
    else:
        return z_min + (thicknesses - thicknesses.min()) / (thicknesses.ptp() + 1e-6) * (z_max - z_min)

def path_to_dataframe(path, z_profile, char_idx):
    """
    단일 path 리스트와 z값, 문자 인덱스를 받아 DataFrame으로 변환  
    path: (x, y) 튜플 리스트  
    z_profile: 각 좌표의 z값 리스트
    char_idx: 문자 인덱스(글자별로 다름)
    """
    xs, ys = zip(*path)
    return pd.DataFrame({
        'char_idx': char_idx,
        'x': xs,
        'y': ys,
        'z': z_profile
    })

def resample_path(df, n_points, max_gap=10.0):
    """
    skeleton 위 점만 등간격으로 뽑으면서,
    두 점 사이 간격이 max_gap 이상이면 path를 끊어서 분리.
    여러 개의 path(DataFrame) 리스트로 반환.
    """
    xs = df['x'].to_numpy()
    ys = df['y'].to_numpy()
    zs = df['z'].to_numpy()
    char_idx = df['char_idx'].iloc[0]

    if len(xs) < 2 or n_points < 2:
        return [df]

    deltas = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2) # 연속된 점 사이 거리
    s = np.concatenate([[0], np.cumsum(deltas)])      
    total_len = s[-1]
    target_s = np.linspace(0, total_len, n_points) # 전체 곡선 n등분

    indices = []
    prev_idx = -1
    for ts in target_s:
        idx = np.argmin(np.abs(s - ts))
        if idx != prev_idx:
            indices.append(idx)
            prev_idx = idx

    # 최대 간격 기준으로 path 분리
    split_indices = [0]
    for i in range(1, len(indices)):
        prev = indices[i-1]
        curr = indices[i]
        dist = np.hypot(xs[curr] - xs[prev], ys[curr] - ys[prev])
        if dist > max_gap:
            split_indices.append(i)  # 이 인덱스부터 새 path 시작

    # 각 path별로 DataFrame 생성
    result_paths = []
    for si, ei in zip(split_indices, split_indices[1:] + [len(indices)]):
        idxs = indices[si:ei]
        if len(idxs) < 2:
            continue
        xs_new = xs[idxs]
        ys_new = ys[idxs]
        zs_new = zs[idxs]
        df_new = pd.DataFrame({
            'char_idx': char_idx,
            'x': xs_new,
            'y': ys_new,
            'z': zs_new
        })
        result_paths.append(df_new)

    return result_paths

class MultiCharacterTrajectoryExtractor:
    """
    여러 글자가 포함된 이미지에서
    각 글자의 binary mask → 중심선(skeleton) → DFS 경로 → 두께 기반 z-profile
    전체를 추출해 DataFrame 형태로 반환하는 클래스
    """
    def __init__(self, img_path, z_min=0.5, z_max=3.0, skeleton_mode='skeletonize'):
        self.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            raise FileNotFoundError(f'이미지 경로를 확인하세요: {img_path}')
        self.binary = binarize_image(self.img)
        self.z_min = z_min
        self.z_max = z_max
        self.skeleton_mode = skeleton_mode
        self.char_dataframes = self.extract_characters()

    def extract_characters(self):
        """
        이진화된 이미지에서 글자별로 contour를 찾고,
        각 contour에 대해:
          - mask 생성
          - 중심선(skeleton) 추출
          - skeleton 경로(들) 추출
          - 각 경로마다 (x, y, z=두께값 정규화) DataFrame 생성
        """
        contours, hierarchy = cv2.findContours(
            self.binary.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        hierarchy = hierarchy[0] if hierarchy is not None else []
        char_dfs = []
        for idx, cnt in enumerate(contours):
            if len(hierarchy) == 0 or hierarchy[idx][3] != -1: # 외곽선이 아닌 내부 구조 (hole)는 무시
                continue
            mask = np.zeros_like(self.binary)
            cv2.drawContours(mask, [cnt], -1, 1, thickness=-1)
            skel = extract_skeleton(mask, self.skeleton_mode)
            if np.count_nonzero(skel) < 10:
                continue
            paths = extract_all_paths(skel)
            for path in paths:
                path = filter_nearby_points(path, min_dist=2.0)
                if len(path) == 0:
                    continue
                path = flip_path_to_start_near_origin(path)
                # mask 로컬 두께 계산 
                thicknesses = [get_local_thickness(y, x, mask) for x, y in path]
                z_profile = normalize_thickness(thicknesses, self.z_min, self.z_max)
                df = path_to_dataframe(path, z_profile, char_idx=idx)
                char_dfs.append(df)
        return char_dfs

    def get_all_dataframes(self):
        return self.char_dataframes

    def get_all_points(self):
        points = []
        for df in self.char_dataframes:
            for _, row in df.iterrows():
                points.append((row['x'], row['y'], row['z'], row['char_idx']))
        return points

    def get_all_dataframes_resampled(self, n_points, max_gap=20.0):
        resampled_nested = [resample_path(df, n_points, max_gap) for df in self.char_dataframes]
        return list(chain.from_iterable(resampled_nested))