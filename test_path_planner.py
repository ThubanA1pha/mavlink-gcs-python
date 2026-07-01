"""
path_planner 모듈 단위 테스트 (GCS / SITL 없이 좌표 리스트만으로 검증).

실행:  python test_path_planner.py
pytest 없이도 동작하도록 작성. 모든 테스트 통과 시 마지막에 ALL PASS 출력.
"""

import math

import path_planner as pp


# 공통 시나리오 좌표 (서울 시청 인근, 동서로 약 530m)
START = (37.5600, 126.9780)
GOAL = (37.5600, 126.9840)
# 직선 경로 한가운데를 막는 사각형 금지구역
BLOCKING_ZONE = [
    (37.5595, 126.9805), (37.5605, 126.9805),
    (37.5605, 126.9815), (37.5595, 126.9815),
]


def _check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"  [PASS] {msg}")


# ------------------------------------------------------------
# 1. 좌표 변환 왕복 (round-trip)
# ------------------------------------------------------------

def test_coordinate_roundtrip():
    print("test_coordinate_roundtrip")
    ref_lat, ref_lon = START
    lat, lon = 37.5611, 126.9799
    x, y = pp.latlon_to_local(lat, lon, ref_lat, ref_lon)
    lat2, lon2 = pp.local_to_latlon(x, y, ref_lat, ref_lon)
    _check(abs(lat - lat2) < 1e-9 and abs(lon - lon2) < 1e-9,
           "위경도 -> 로컬 -> 위경도 왕복 오차 < 1e-9")
    # 동쪽 1° 경도가 대략 cos(lat)*111320 m 인지 (근사 확인)
    xe, ye = pp.latlon_to_local(ref_lat, ref_lon + 1.0, ref_lat, ref_lon)
    expected = 111320.0 * math.cos(math.radians(ref_lat))
    _check(abs(xe - expected) < 1.0, "경도 1° -> 동쪽 미터 근사 일치")


# ------------------------------------------------------------
# 2. 점-다각형 판정 (ray casting)
# ------------------------------------------------------------

def test_point_in_polygon():
    print("test_point_in_polygon")
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    _check(pp.point_in_polygon(5, 5, square), "중심점은 내부")
    _check(not pp.point_in_polygon(15, 5, square), "오른쪽 바깥점은 외부")
    _check(not pp.point_in_polygon(-1, -1, square), "왼쪽 아래 바깥점은 외부")
    _check(not pp.point_in_polygon(5, 5, [(0, 0), (10, 0)]),
           "정점 2개(면적 없음)는 항상 외부")


# ------------------------------------------------------------
# 3. 장애물 없을 때: 거의 직선 (평활화 후 2점)
# ------------------------------------------------------------

def test_no_obstacle_is_straight():
    print("test_no_obstacle_is_straight")
    res = pp.plan_path(START, GOAL, [], cell_size_m=5.0, safety_margin_m=0.0)
    _check(res["success"], "장애물 없으면 경로 성공")
    _check(len(res["waypoints"]) == 2,
           f"평활화 후 웨이포인트 2개 (start,goal) (실제 {len(res['waypoints'])})")
    m = res["metrics"]
    _check(abs(m["path_length_smoothed_m"] - m["straight_line_m"]) < m["cell_size_m"],
           "평활화 경로 길이 ~= 직선 거리")


# ------------------------------------------------------------
# 4. 장애물 있을 때: 회피 성공 + 구역 침범 없음
# ------------------------------------------------------------

def test_avoids_blocking_zone():
    print("test_avoids_blocking_zone")
    res = pp.plan_path(START, GOAL, [BLOCKING_ZONE],
                       cell_size_m=5.0, safety_margin_m=10.0)
    _check(res["success"], "막힌 직선이라도 우회 경로 성공")
    _check(not pp.path_intersects_zones(res["waypoints"], [BLOCKING_ZONE]),
           "생성된 경로가 금지구역을 침범하지 않음")
    m = res["metrics"]
    _check(m["path_length_smoothed_m"] > m["straight_line_m"],
           "우회 경로는 직선보다 길다")


# ------------------------------------------------------------
# 5. 직선 GoTo는 구역을 관통한다 (필연성 증명용 대조)
# ------------------------------------------------------------

def test_straight_line_hits_zone():
    print("test_straight_line_hits_zone")
    straight = [START, GOAL]
    _check(pp.path_intersects_zones(straight, [BLOCKING_ZONE]),
           "직선 GoTo는 금지구역을 관통 (대조군)")


# ------------------------------------------------------------
# 6. 평활화가 웨이포인트 수를 줄인다
# ------------------------------------------------------------

def test_smoothing_reduces_waypoints():
    print("test_smoothing_reduces_waypoints")
    res = pp.plan_path(START, GOAL, [BLOCKING_ZONE],
                       cell_size_m=5.0, safety_margin_m=10.0)
    m = res["metrics"]
    _check(m["waypoints_smoothed"] <= m["waypoints_raw"],
           f"평활화 후 <= 전 ({m['waypoints_smoothed']} <= {m['waypoints_raw']})")
    _check(m["waypoints_smoothed"] < m["waypoints_raw"],
           "이번 시나리오에선 평활화로 실제 감소")


# ------------------------------------------------------------
# 7. 엣지케이스: 목표가 금지구역 안 -> 실패
# ------------------------------------------------------------

def test_goal_inside_zone_fails():
    print("test_goal_inside_zone_fails")
    goal_inside = (37.5600, 126.9810)  # BLOCKING_ZONE 내부
    res = pp.plan_path(START, goal_inside, [BLOCKING_ZONE],
                       cell_size_m=5.0, safety_margin_m=10.0)
    _check(not res["success"], "목표가 구역 내부면 실패")
    _check("목표" in res["reason"], f"실패 사유에 '목표' 포함 (실제: {res['reason']})")


# ------------------------------------------------------------
# 8. 엣지케이스: 정점 2개 구역은 무시
# ------------------------------------------------------------

def test_degenerate_zone_ignored():
    print("test_degenerate_zone_ignored")
    degenerate = [(37.5598, 126.9805), (37.5602, 126.9815)]  # 정점 2개
    res = pp.plan_path(START, GOAL, [degenerate],
                       cell_size_m=5.0, safety_margin_m=0.0)
    _check(res["success"], "면적 없는 구역은 무시하고 성공")
    _check(len(res["waypoints"]) == 2, "무시되었으므로 직선(2점)")


# ------------------------------------------------------------
# 9. 격자 셀 수 상한 -> 해상도 자동 완화
# ------------------------------------------------------------

def test_max_cells_coarsens():
    print("test_max_cells_coarsens")
    res = pp.plan_path(START, GOAL, [], cell_size_m=0.2,  # 매우 촘촘 -> 상한 초과 유도
                       safety_margin_m=0.0, max_cells=10_000)
    _check(res["success"], "셀 상한 초과해도 자동 완화 후 성공")
    m = res["metrics"]
    _check(m["total_cells"] <= 10_000, f"셀 수 상한 준수 ({m['total_cells']})")
    _check(m["cell_size_m"] > m["requested_cell_size_m"], "해상도가 자동 완화됨")


def main():
    tests = [
        test_coordinate_roundtrip,
        test_point_in_polygon,
        test_no_obstacle_is_straight,
        test_avoids_blocking_zone,
        test_straight_line_hits_zone,
        test_smoothing_reduces_waypoints,
        test_goal_inside_zone_fails,
        test_degenerate_zone_ignored,
        test_max_cells_coarsens,
    ]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            failed += 1
            print(f"  [FAIL] {e}")
        except Exception as e:
            failed += 1
            print(f"  [ERROR] {t.__name__}: {e}")
    print("-" * 50)
    if failed == 0:
        print(f"ALL PASS ({len(tests)} tests)")
    else:
        print(f"{failed} / {len(tests)} FAILED")
    return failed


if __name__ == "__main__":
    raise SystemExit(main())
