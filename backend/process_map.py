import json
import argparse
import numpy as np
from pathlib import Path

def create_road_polygon(points, width):
    """
    도로 중심선(points)과 폭(width)을 기반으로 도로 표면에 해당하는 Polygon 좌표를 생성합니다.
    (이 함수 내용은 이전과 동일)
    """
    centerline = np.array(points, dtype=float)
    right_boundary, left_boundary = [], []
    half_width = width / 2.0

    for i in range(len(centerline) - 1):
        p1, p2 = centerline[i], centerline[i+1]
        direction_vector = p2 - p1
        if np.linalg.norm(direction_vector) == 0: continue
        normal_vector = np.array([direction_vector[1], -direction_vector[0]])
        unit_normal = normal_vector / np.linalg.norm(normal_vector)

        right_p1, left_p1 = p1 + unit_normal * half_width, p1 - unit_normal * half_width
        right_p2, left_p2 = p2 + unit_normal * half_width, p2 - unit_normal * half_width

        if i == 0:
            right_boundary.append(right_p1.tolist())
            left_boundary.append(left_p1.tolist())
        right_boundary.append(right_p2.tolist())
        left_boundary.append(left_p2.tolist())

    if not right_boundary or not left_boundary: return None
    polygon_coords = right_boundary + left_boundary[::-1] + [right_boundary[0]]
    return [polygon_coords]


def main():
    """메인 실행 함수"""
    parser = argparse.ArgumentParser(
        description="도로 중심선(link) JSON 데이터를 도로 표면(Polygon) GeoJSON으로 변환합니다."
    )
    parser.add_argument("--input", type=str, required=True, help="입력할 link_set.json 파일 경로")
    parser.add_argument("--output", type=str, required=True, help="출력할 merged_road_surfaces.geojson 파일 경로")
    parser.add_argument("--width", type=float, default=5.0, help="기본 도로 폭 (미터 단위, 기본값: 5.0)")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    default_width = args.width

    if not input_path.is_file():
        print(f"오류: 입력 파일 '{input_path}'을(를) 찾을 수 없습니다.")
        return

    with open(input_path, 'r', encoding='utf-8') as f:
        link_data = json.load(f)

    geojson_features = []
    
    features = None
    if isinstance(link_data, list):
        features = link_data
    elif isinstance(link_data, dict):
        features = link_data.get('features')
        if not features:
            features = link_data.get('links')
    
    if features is None:
        print(f"오류: 입력 파일에서 도로 데이터(features 또는 links)를 찾을 수 없습니다. 데이터 형식을 확인해주세요.")
        return

    for feature in features:
        # --- ⬇️ (중요) 이 부분이 변경되었습니다 ⬇️ ---
        
        points = None
        geometry = feature.get('geometry')

        # 데이터 구조에 따라 유연하게 points를 추출합니다.
        if isinstance(geometry, dict) and geometry.get('type') == 'LineString':
            # 1. 표준 GeoJSON 형식: geometry가 딕셔너리인 경우
            points = geometry.get('coordinates')
        elif isinstance(geometry, list):
            # 2. 오류 발생 형식: geometry가 리스트인 경우
            points = geometry
        else:
            # 3. 이전 형식: 'points' 키를 직접 사용하는 경우
            points = feature.get('points')

        if not points or len(points) < 2:
            continue
        
        # --- ⬆️ (중요) 이 부분이 변경되었습니다 ⬆️ ---
        
        properties = feature.get('properties', {})
        road_width = feature.get('width', properties.get('width', default_width))
        link_id = feature.get('link_id', properties.get('link_id', "N/A"))

        polygon_coordinates = create_road_polygon(points, road_width)
        
        if polygon_coordinates:
            new_feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": polygon_coordinates
                },
                "properties": { "link_id": link_id }
            }
            geojson_features.append(new_feature)

    geojson_output = {
        "type": "FeatureCollection",
        "features": geojson_features
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(geojson_output, f, indent=2)

    print(f"✅ 성공: {len(geojson_features)}개의 도로를 처리하여 '{output_path}' 파일로 저장했습니다.")


if __name__ == "__main__":
    main()

