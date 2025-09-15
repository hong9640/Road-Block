import json

# 원본 파일 경로와 결과 파일 경로
src_path = "link_set.json"          # 입력 파일
dst_path = "link_set_2d.json"       # 출력 파일

# JSON 불러오기
with open(src_path, "r", encoding="utf-8") as f:
    data = json.load(f)

# z축 제거 함수: [x, y, z] → [x, y]
def drop_z(points):
    return [[p[0], p[1]] for p in points if isinstance(p, list) and len(p) >= 2]

# 레코드 변환
def convert(obj):
    rec = dict(obj)
    if "points" in rec:
        pts = rec["points"]
        if isinstance(pts, list) and pts:
            # points가 [x,y,z] 리스트인 경우
            if isinstance(pts[0], list) and all(isinstance(x, (int, float)) for x in pts[0]):
                rec["points"] = drop_z(pts)
    return rec

# 데이터 구조 변환 (리스트 또는 {links:[...]} 구조 지원)
if isinstance(data, list):
    converted = [convert(item) for item in data]
elif isinstance(data, dict) and "links" in data:
    converted = dict(data)
    converted["links"] = [convert(item) for item in data["links"]]
else:
    converted = convert(data)

# 결과 저장
with open(dst_path, "w", encoding="utf-8") as f:
    json.dump(converted, f, ensure_ascii=False, indent=2)

print(f"변환 완료 → {dst_path}")
