import numpy as np
from collections import deque

def extract_all_paths(skeleton, sort_by='x', ascending=True):
    """
    2D binary skeleton 이미지(값이 1인 픽셀 집합)에서  
    연결된 모든 중심선 path(궤적)를 DFS(깊이 우선 탐색)로 추출합니다.
    
    Args:
        skeleton (np.ndarray): 2D numpy array, 중심선은 값 1, 배경은 0
    
    Returns:
        paths (list of list of tuple):  
            각 path는 (x, y) 튜플의 ordered 리스트  
            여러 개의 path(글자 분기, 여러 문자 등) 반환
    """
    ys, xs = np.where(skeleton == 1)             # 중심선(값 1) 픽셀의 좌표 집합
    skel_points = set(zip(xs, ys))               # (x, y) 튜플 set으로 변환 (빠른 탐색 위해)
    visited = set()                              # 방문한 점 저장
    paths = []                                   # 최종 결과로 반환할 path 리스트

    def neighbors(x, y):
        """
        (x, y) 기준으로 8방향 이웃 중,
        아직 방문하지 않은 skeleton 픽셀 좌표를 generator로 반환
        """
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue    # 자기 자신은 제외
                nx, ny = x + dx, y + dy
                if (nx, ny) in skel_points and (nx, ny) not in visited:
                    yield nx, ny

    # 방문하지 않은 skeleton 픽셀들이 남아있는 동안 반복
    while skel_points - visited:
        # ---- 여기서 정렬 ----
        unvisited = list(skel_points - visited)
        if sort_by == 'x':
            unvisited.sort(key=lambda pt: pt[0], reverse=not ascending)
        elif sort_by == 'y':
            unvisited.sort(key=lambda pt: pt[1], reverse=not ascending)
        # -------------------
        start = unvisited[0]           # 방문하지 않은 픽셀 하나 선택 (새 path의 시작점)
        path = [start]                 # 현재 path 저장 (ordered)
        visited.add(start)             # 시작점 방문 처리
        dq = deque([start])            # DFS를 위한 스택 구조(후입선출)

        while dq:
            curr = dq.pop()
            for nxt in neighbors(*curr):
                if nxt not in visited:
                    path.append(nxt)
                    visited.add(nxt)
                    dq.append(nxt)
        if len(path) > 1:
            paths.append(path)         # 2개 이상 연결된 경로만 결과에 추가

    return paths