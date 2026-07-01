"""
비행금지구역(No-Fly Zone) 회피 경로계획 모듈 (GCS와 분리된 순수 모듈).

흐름:
  1) 위경도 <-> 로컬 평면 미터 변환 (작업 영역이 좁으므로 평면 근사)
  2) 드론 위치 ~ 목표점을 감싸는 bounding box를 해상도 r로 격자화
  3) 각 셀 중심에 점-다각형 판정(ray casting) + 안전여유(margin) 팽창으로 차단 셀 표시
  4) 8방향 A* 탐색(octile 휴리스틱, corner-cutting 방지)
  5) string-pulling 평활화(가시선 검사)로 웨이포인트 수 축소
  6) 평활화된 셀 경로를 위경도 웨이포인트로 환산해 반환

이 모듈은 GCS / SITL / tkinter에 의존하지 않는다. (단위 테스트·논문 측정 용이)
"""

import math
import time
import heapq


# ============================================================
# 1. 좌표 변환 유틸 (위경도 <-> 로컬 평면 미터)
# ============================================================

METERS_PER_DEG_LAT = 111_320.0


def latlon_to_local(lat, lon, ref_lat, ref_lon):
    """기준점(ref) 기준으로 (lat, lon)을 로컬 평면 미터 (x=동쪽, y=북쪽)로 변환."""
    m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    x = (lon - ref_lon) * m_per_deg_lon
    y = (lat - ref_lat) * METERS_PER_DEG_LAT
    return x, y


def local_to_latlon(x, y, ref_lat, ref_lon):
    """로컬 평면 미터 (x, y)를 위경도로 환산."""
    m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    lat = ref_lat + y / METERS_PER_DEG_LAT
    lon = ref_lon + x / m_per_deg_lon
    return lat, lon


# ============================================================
# 2. 기하 판정 (점-다각형, 점-선분 거리)
# ============================================================

def point_in_polygon(px, py, polygon):
    """
    ray casting 알고리즘으로 점 (px, py)가 다각형 내부에 있는지 판정.
    polygon: [(x, y), ...] (로컬 미터 좌표). 정점 3개 미만이면 False.
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        # 수평 ray가 변 (i, j)와 교차하는지
        if ((yi > py) != (yj > py)):
            x_cross = (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def _point_segment_distance(px, py, ax, ay, bx, by):
    """점 (px, py)에서 선분 (a-b)까지의 최단 거리."""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def distance_to_polygon(px, py, polygon):
    """점 (px, py)에서 다각형 경계(변)까지의 최단 거리. 정점 3개 미만이면 무한대."""
    n = len(polygon)
    if n < 3:
        return float("inf")
    best = float("inf")
    j = n - 1
    for i in range(n):
        d = _point_segment_distance(px, py, polygon[i][0], polygon[i][1],
                                    polygon[j][0], polygon[j][1])
        if d < best:
            best = d
        j = i
    return best


def _blocked_at(px, py, polygons, margin):
    """점 (px, py)가 어떤 금지구역 내부이거나 경계로부터 margin 이내면 True."""
    for poly in polygons:
        if len(poly) < 3:
            continue
        if point_in_polygon(px, py, poly):
            return True
        if margin > 0 and distance_to_polygon(px, py, poly) < margin:
            return True
    return False


# ============================================================
# 3. 격자(Grid) 구성
# ============================================================

SQRT2 = math.sqrt(2.0)


class Grid:
    """격자화된 작업 영역과 차단 셀 정보를 담는다."""

    def __init__(self, min_x, min_y, cell_size, cols, rows):
        self.min_x = min_x
        self.min_y = min_y
        self.cell = cell_size
        self.cols = cols
        self.rows = rows
        # blocked[j][i] = True 면 통행 불가
        self.blocked = [[False] * cols for _ in range(rows)]
        self.blocked_count = 0

    def cell_center(self, i, j):
        """셀 (i, j)의 중심 로컬 좌표."""
        return (self.min_x + (i + 0.5) * self.cell,
                self.min_y + (j + 0.5) * self.cell)

    def point_to_cell(self, x, y):
        """로컬 좌표 (x, y)가 속한 셀 (i, j) (격자 범위로 클램프)."""
        i = int((x - self.min_x) / self.cell)
        j = int((y - self.min_y) / self.cell)
        i = max(0, min(self.cols - 1, i))
        j = max(0, min(self.rows - 1, j))
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.cols and 0 <= j < self.rows

    def is_blocked(self, i, j):
        return self.blocked[j][i]


def build_grid(start_local, goal_local, polygons_local,
               cell_size, margin, bbox_pad, max_cells):
    """
    start/goal/금지구역을 모두 감싸는 bounding box를 만들고 격자화한다.
    셀 수가 max_cells를 넘으면 해상도를 자동 완화(coarsen)한다.
    반환: (Grid, 실제 사용된 cell_size)
    """
    xs = [start_local[0], goal_local[0]]
    ys = [start_local[1], goal_local[1]]
    for poly in polygons_local:
        for (x, y) in poly:
            xs.append(x)
            ys.append(y)

    min_x = min(xs) - bbox_pad
    max_x = max(xs) + bbox_pad
    min_y = min(ys) - bbox_pad
    max_y = max(ys) + bbox_pad

    width = max(max_x - min_x, cell_size)
    height = max(max_y - min_y, cell_size)

    # 셀 수 상한 초과 시 해상도 자동 완화
    used_cell = float(cell_size)
    for _ in range(20):
        cols = max(1, int(math.ceil(width / used_cell)))
        rows = max(1, int(math.ceil(height / used_cell)))
        if cols * rows <= max_cells:
            break
        used_cell *= 1.5

    cols = max(1, int(math.ceil(width / used_cell)))
    rows = max(1, int(math.ceil(height / used_cell)))

    grid = Grid(min_x, min_y, used_cell, cols, rows)

    # 차단 셀 마킹
    count = 0
    for j in range(rows):
        for i in range(cols):
            cx, cy = grid.cell_center(i, j)
            if _blocked_at(cx, cy, polygons_local, margin):
                grid.blocked[j][i] = True
                count += 1
    grid.blocked_count = count
    return grid, used_cell


# ============================================================
# 4. A* 탐색 + 가시선(LOS) + 평활화
# ============================================================

_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1),
              (-1, -1), (-1, 1), (1, -1), (1, 1)]


def astar(grid, start_cell, goal_cell):
    """
    8방향 A* (octile 휴리스틱). 대각선 이동 시 양옆 셀이 막혀 있으면 통과 금지.
    반환: 셀 (i, j) 리스트 (시작 -> 목표), 경로 없으면 None.
    """
    si, sj = start_cell
    gi, gj = goal_cell

    if grid.is_blocked(si, sj) or grid.is_blocked(gi, gj):
        return None

    def heuristic(i, j):
        dx = abs(i - gi)
        dy = abs(j - gj)
        return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

    open_heap = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(si, sj), counter, (si, sj)))
    came_from = {}
    g_score = {(si, sj): 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == (gi, gj):
            return _reconstruct(came_from, current)
        closed.add(current)

        ci, cj = current
        for di, dj in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if not grid.in_bounds(ni, nj):
                continue
            if grid.is_blocked(ni, nj):
                continue
            if di != 0 and dj != 0:
                # corner-cutting 방지: 대각선 이동 시 양옆 셀이 막혀 있으면 금지
                if grid.is_blocked(ci + di, cj) or grid.is_blocked(ci, cj + dj):
                    continue
                step = SQRT2
            else:
                step = 1.0

            neighbor = (ni, nj)
            tentative = g_score[current] + step
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                counter += 1
                f = tentative + heuristic(ni, nj)
                heapq.heappush(open_heap, (f, counter, neighbor))

    return None


def _reconstruct(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def nearest_free_cell(grid, cell):
    """시작 셀이 막혀 있을 때, BFS로 가장 가까운 자유(비차단) 셀을 찾는다."""
    from collections import deque
    si, sj = cell
    if not grid.is_blocked(si, sj):
        return cell
    visited = {cell}
    q = deque([cell])
    while q:
        ci, cj = q.popleft()
        for di, dj in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if not grid.in_bounds(ni, nj) or (ni, nj) in visited:
                continue
            visited.add((ni, nj))
            if not grid.is_blocked(ni, nj):
                return (ni, nj)
            q.append((ni, nj))
    return None


def nearest_safe_point(current, no_fly_zones,
                       cell_size_m=5.0, safety_margin_m=10.0, bbox_pad_m=80.0):
    """
    현재 위치(current=(lat,lon))가 금지구역/위협반경 '내부 또는 안전여유 안'일 때,
    가장 가까운 '구역+안전여유 밖' 지점을 (lat, lon)로 반환한다.
    이미 안전하면 current를 그대로, 탈출 지점을 못 찾으면 None을 반환한다.

    (구역 한가운데처럼 깊숙이 진입한 경우 plan_path의 목표가 막혀 실패하는 문제를
     보강하기 위한 함수. 내부적으로 build_grid + nearest_free_cell을 재사용한다.)
    """
    ref_lat, ref_lon = current
    polygons_local = []
    for zone in no_fly_zones:
        if len(zone) < 3:
            continue
        polygons_local.append([latlon_to_local(la, lo, ref_lat, ref_lon)
                               for (la, lo) in zone])
    if not polygons_local:
        return current

    cur_local = (0.0, 0.0)  # 기준점이 현재 위치
    if not _blocked_at(cur_local[0], cur_local[1], polygons_local, safety_margin_m):
        return current  # 이미 안전 지대

    grid, _ = build_grid(cur_local, cur_local, polygons_local,
                         cell_size_m, safety_margin_m, bbox_pad_m, 200_000)
    cur_cell = grid.point_to_cell(*cur_local)
    free = nearest_free_cell(grid, cur_cell)
    if free is None:
        return None
    fx, fy = grid.cell_center(*free)
    return local_to_latlon(fx, fy, ref_lat, ref_lon)


def line_of_sight(grid, c0, c1):
    """두 셀을 잇는 직선이 차단 셀을 통과하지 않으면 True (셀 중심 기준 샘플링)."""
    i0, j0 = c0
    i1, j1 = c1
    di, dj = i1 - i0, j1 - j0
    dist = math.hypot(di, dj)
    # 셀당 4회 샘플링 (얇은 장애물도 잡히도록 촘촘히)
    steps = int(dist * 4) + 1
    for k in range(steps + 1):
        t = k / steps
        ci = int(round(i0 + di * t))
        cj = int(round(j0 + dj * t))
        if not grid.in_bounds(ci, cj) or grid.is_blocked(ci, cj):
            return False
    return True


def smooth_path(grid, cell_path):
    """
    string-pulling 평활화: 가시선으로 직접 이을 수 있는 중간 셀을 제거한다.
    반환: 평활화된 셀 리스트.
    """
    if len(cell_path) <= 2:
        return list(cell_path)

    smoothed = [cell_path[0]]
    i = 0
    n = len(cell_path)
    while i < n - 1:
        j = n - 1
        # i에서 가장 멀리 보이는 점 j를 찾는다.
        while j > i + 1:
            if line_of_sight(grid, cell_path[i], cell_path[j]):
                break
            j -= 1
        smoothed.append(cell_path[j])
        i = j
    return smoothed


# ============================================================
# 5. 통합: plan_path
# ============================================================

def _path_length_local(points_local):
    """로컬 좌표 점 리스트의 총 길이(m)."""
    total = 0.0
    for a, b in zip(points_local, points_local[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def plan_path(start, goal, no_fly_zones,
              cell_size_m=5.0, safety_margin_m=10.0,
              bbox_pad_m=50.0, max_cells=200_000):
    """
    비행금지구역을 회피하는 경로를 계획한다.

    인자:
      start, goal     : (lat, lon)
      no_fly_zones    : 금지구역 리스트. 각 구역 = [(lat, lon), ...] (정점 3개 이상)
      cell_size_m     : 격자 해상도 r (미터/셀)
      safety_margin_m : 안전여유. 구역 경계로부터 이 거리 안쪽 셀도 차단
      bbox_pad_m      : 작업 영역 여백(미터)
      max_cells       : 셀 수 상한 (초과 시 해상도 자동 완화)

    반환(dict):
      success         : bool
      reason          : 실패 사유 문자열 (성공 시 "")
      waypoints       : [(lat, lon), ...] 평활화된 경로 (시작~목표, 정확한 끝점 포함)
      raw_waypoints   : [(lat, lon), ...] 평활화 전 격자 경로
      metrics         : 측정 지표 dict
    """
    t_start = time.perf_counter()

    ref_lat, ref_lon = start
    start_local = (0.0, 0.0)  # 기준점이 출발점이므로
    goal_local = latlon_to_local(goal[0], goal[1], ref_lat, ref_lon)

    polygons_local = []
    for zone in no_fly_zones:
        if len(zone) < 3:
            continue  # 면적 없는 구역은 무시
        polygons_local.append([latlon_to_local(la, lo, ref_lat, ref_lon)
                               for (la, lo) in zone])

    result = {
        "success": False,
        "reason": "",
        "waypoints": [],
        "raw_waypoints": [],
        "metrics": {},
    }

    # 엣지케이스: 목표점이 금지구역 내부
    for poly in polygons_local:
        if point_in_polygon(goal_local[0], goal_local[1], poly):
            result["reason"] = "목표 지점이 금지구역 안에 있습니다."
            result["metrics"]["planning_time_s"] = time.perf_counter() - t_start
            return result

    grid, used_cell = build_grid(start_local, goal_local, polygons_local,
                                 cell_size_m, safety_margin_m, bbox_pad_m, max_cells)

    start_cell = grid.point_to_cell(*start_local)
    goal_cell = grid.point_to_cell(*goal_local)

    # 엣지케이스: 목표 셀이 (안전여유로) 차단된 경우
    if grid.is_blocked(*goal_cell):
        result["reason"] = "목표 지점이 안전여유 구역 안에 있어 접근할 수 없습니다."
        result["metrics"]["planning_time_s"] = time.perf_counter() - t_start
        return result

    # 엣지케이스: 시작 셀이 차단 → 가장 가까운 자유 셀로 보정
    start_corrected = False
    if grid.is_blocked(*start_cell):
        free = nearest_free_cell(grid, start_cell)
        if free is None:
            result["reason"] = "출발점이 완전히 봉쇄되어 있습니다."
            result["metrics"]["planning_time_s"] = time.perf_counter() - t_start
            return result
        start_cell = free
        start_corrected = True

    cell_path = astar(grid, start_cell, goal_cell)
    if cell_path is None:
        result["reason"] = "경로를 찾을 수 없습니다 (완전 봉쇄)."
        result["metrics"]["planning_time_s"] = time.perf_counter() - t_start
        return result

    smoothed_cells = smooth_path(grid, cell_path)

    # 셀 -> 로컬 -> 위경도 환산. 끝점은 정확한 start/goal로 치환.
    def cells_to_latlon(cells, exact_start, exact_goal):
        pts = [grid.cell_center(i, j) for (i, j) in cells]
        latlon = [local_to_latlon(x, y, ref_lat, ref_lon) for (x, y) in pts]
        if latlon:
            latlon[0] = exact_start
            latlon[-1] = exact_goal
        return latlon

    raw_latlon = cells_to_latlon(cell_path, start, goal)
    smooth_latlon = cells_to_latlon(smoothed_cells, start, goal)

    # 길이 측정(로컬)
    raw_local = [grid.cell_center(i, j) for (i, j) in cell_path]
    smooth_local = [grid.cell_center(i, j) for (i, j) in smoothed_cells]
    if smooth_local:
        smooth_local[0] = start_local
        smooth_local[-1] = goal_local

    result["success"] = True
    result["waypoints"] = smooth_latlon
    result["raw_waypoints"] = raw_latlon
    result["metrics"] = {
        "planning_time_s": time.perf_counter() - t_start,
        "grid_cols": grid.cols,
        "grid_rows": grid.rows,
        "total_cells": grid.cols * grid.rows,
        "blocked_cells": grid.blocked_count,
        "cell_size_m": used_cell,
        "requested_cell_size_m": cell_size_m,
        "safety_margin_m": safety_margin_m,
        "waypoints_raw": len(cell_path),
        "waypoints_smoothed": len(smoothed_cells),
        "path_length_raw_m": _path_length_local(raw_local),
        "path_length_smoothed_m": _path_length_local(smooth_local),
        "straight_line_m": math.hypot(goal_local[0], goal_local[1]),
        "start_corrected": start_corrected,
    }
    return result


# ============================================================
# 6. 보조: 경로가 금지구역을 침범하는지 검사 (검증/논문 측정용)
# ============================================================

def path_intersects_zones(waypoints_latlon, no_fly_zones, samples_per_seg=40):
    """
    웨이포인트 경로(위경도)가 어떤 금지구역이라도 통과하면 True.
    각 선분을 samples_per_seg 개로 샘플링해 점-다각형 판정.
    (안전여유 margin은 고려하지 않고 '실제 구역 내부' 진입만 검사)
    """
    if len(waypoints_latlon) < 2 or not no_fly_zones:
        return False

    ref_lat, ref_lon = waypoints_latlon[0]
    polys_local = [[latlon_to_local(la, lo, ref_lat, ref_lon) for (la, lo) in z]
                   for z in no_fly_zones if len(z) >= 3]

    pts_local = [latlon_to_local(la, lo, ref_lat, ref_lon) for (la, lo) in waypoints_latlon]
    for a, b in zip(pts_local, pts_local[1:]):
        for k in range(samples_per_seg + 1):
            t = k / samples_per_seg
            px = a[0] + (b[0] - a[0]) * t
            py = a[1] + (b[1] - a[1]) * t
            for poly in polys_local:
                if point_in_polygon(px, py, poly):
                    return True
    return False


if __name__ == "__main__":
    # 간단 데모: 직선 경로를 막는 사각형 금지구역을 우회
    start = (37.5600, 126.9780)
    goal = (37.5600, 126.9840)   # 동쪽으로 약 530m
    # 직선 경로 한가운데를 막는 사각형
    zone = [(37.5595, 126.9805), (37.5605, 126.9805),
            (37.5605, 126.9815), (37.5595, 126.9815)]

    res = plan_path(start, goal, [zone], cell_size_m=5.0, safety_margin_m=10.0)
    print("success:", res["success"], "| reason:", res["reason"])
    if res["success"]:
        m = res["metrics"]
        print(f"격자: {m['grid_cols']}x{m['grid_rows']} (cell={m['cell_size_m']:.1f}m), "
              f"차단셀={m['blocked_cells']}")
        print(f"웨이포인트: raw={m['waypoints_raw']} -> smoothed={m['waypoints_smoothed']}")
        print(f"경로 길이: raw={m['path_length_raw_m']:.1f}m, "
              f"smoothed={m['path_length_smoothed_m']:.1f}m, "
              f"직선={m['straight_line_m']:.1f}m")
        print(f"계획 시간: {m['planning_time_s']*1000:.1f}ms")
        print("구역 침범:", path_intersects_zones(res["waypoints"], [zone]))
