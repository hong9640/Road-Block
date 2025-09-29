import json
import math
from typing import List, Tuple, Any

# -----------------------------
# Ramer–Douglas–Peucker (RDP)
# -----------------------------
def _perp_distance(point: Tuple[float, float],
                   start: Tuple[float, float],
                   end: Tuple[float, float]) -> float:
    """점 point와 선분 start-end 사이의 수직거리(미터)를 계산."""
    x, y = point
    x1, y1 = start
    x2, y2 = end
    dx, dy = (x2 - x1), (y2 - y1)
    if dx == 0 and dy == 0:
        # 퇴화: start == end
        return math.hypot(x - x1, y - y1)
    # 선분의 매개변수 t (투영)
    t = ((x - x1) * dx + (y - y1) * dy) / (dx*dx + dy*dy)
    if t < 0.0:
        # start 쪽으로 가장 가까움
        return math.hypot(x - x1, y - y1)
    elif t > 1.0:
        # end 쪽으로 가장 가까움
        return math.hypot(x - x2, y - y2)
    # 선분 위 최근접점
    projx = x1 + t * dx
    projy = y1 + t * dy
    return math.hypot(x - projx, y - projy)

def rdp(points: List[List[float]], epsilon: float) -> List[List[float]]:
    """
    RDP 알고리즘: 폴리라인을 허용오차 epsilon(미터)로 단순화.
    - endpoints(양 끝점) 보존
    - 선형 복잡도 ~ O(n log n) 수준의 분할/정복 재귀(최악 O(n^2))
    """
    if len(points) <= 2:
        return points[:]

    # 가장 멀리 떨어진 점 찾기
    start = tuple(points[0])
    end   = tuple(points[-1])
    max_dist = -1.0
    index = -1
    for i in range(1, len(points) - 1):
        d = _perp_distance(tuple(points[i]), start, end)
        if d > max_dist:
            max_dist = d
            index = i

    # 최대 수직거리가 epsilon보다 크면, 해당 점에서 분할하여 재귀
    if max_dist > epsilon:
        left  = rdp(points[:index+1], epsilon)
        right = rdp(points[index:],   epsilon)
        return left[:-1] + right      # 중복되는 분할점 제거
    else:
        # 선분으로 충분히 근사 가능 → 시작/끝만 남김
        return [points[0], points[-1]]

# -----------------------------
# (선택) 아주 촘촘한 연속점(예: < min_gap m) 사전 정리
# -----------------------------
def dedupe_dense(points: List[List[float]], min_gap: float) -> List[List[float]]:
    """
    연속 점 간 거리가 min_gap 미만이면 스킵하여 군집 제거.
    - RDP 이전에 적용하면 성능/결과 안정성 향상
    """
    if not points:
        return points
    out = [points[0]]
    last = points[0]
    for p in points[1:]:
        if math.hypot(p[0]-last[0], p[1]-last[1]) >= min_gap:
            out.append(p)
            last = p
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out

# -----------------------------
# 파일 변환 유틸
# -----------------------------
def simplify_record(rec: dict, epsilon: float, min_gap: float = 0.0) -> dict:
    """
    단일 링크 레코드의 'points'를 단순화하고 나머지 필드는 보존.
    - epsilon: RDP 허용오차(미터)
    - min_gap: 연속 점 간 최소 간격(미터). 0이면 비활성.
    """
    out = dict(rec)
    pts = rec.get("points")
    if isinstance(pts, list) and len(pts) >= 2 and isinstance(pts[0], list):
        pts2 = pts
        if min_gap > 0.0:
            pts2 = dedupe_dense(pts2, min_gap=min_gap)
        if len(pts2) >= 2:
            pts2 = rdp(pts2, epsilon=epsilon)
        out["points"] = pts2
    return out

def simplify_json(src_path: str, dst_path: str,
                  epsilon: float = 0.10,   # 10cm 허용오차(필요 시 조정)
                  min_gap: float = 0.00    # 매우 촘촘한 연속점 제거(옵션)
                 ) -> None:
    with open(src_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, list):
        links = data
        simplified = [simplify_record(link, epsilon, min_gap) for link in links]
    elif isinstance(data, dict) and "links" in data:
        simplified = dict(data)
        simplified["links"] = [
            simplify_record(link, epsilon, min_gap) for link in data["links"]
        ]
    else:
        simplified = simplify_record(data, epsilon, min_gap)

    with open(dst_path, "w", encoding="utf-8") as f:
        json.dump(simplified, f, ensure_ascii=False, indent=2)

# -----------------------------
# 실행 예시
# -----------------------------
if __name__ == "__main__":
    # 입력/출력 파일 경로를 맞춰 주세요.
    src = "link_set_2d.json"           # 2D 데이터(JSON)
    dst = "link_set_2d_simplified.json"

    # 허용오차 ε와 연속점 최소 간격 설정(단위: m)
    epsilon = 0.10   # 10 cm 이내 오차 허용
    min_gap = 0.00   # 필요 시 0.02(2 cm) 등으로 설정

    simplify_json(src, dst, epsilon=epsilon, min_gap=min_gap)
    print(f"Simplified -> {dst} (epsilon={epsilon}, min_gap={min_gap})")
