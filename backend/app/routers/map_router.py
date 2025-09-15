from pathlib import Path
from fastapi import APIRouter, HTTPException, status
from fastapi.responses import FileResponse

# --- 상수 및 라우터 설정 ---

# 'app/maps' 디렉토리에 맵 데이터가 있다고 가정합니다.
MAP_DATA_BASE_PATH = Path("app/maps/")

# DB를 대신하는 간단한 맵 정보
# 각 폴더 안에는 미리 처리된 'merged_road_surfaces.geojson' 파일이 있어야 합니다.
MAP_METADATA = {
    1: {"folder_name": "R_KR_PG_K-City", "map_name": "K-City"},
    2: {"folder_name": "R_KR_PG_KATRI", "map_name": "KATRI"},
    3: {"folder_name": "R_KR_PG_KIAPI", "map_name": "KIAPI"},
    4: {"folder_name": "R_KR_PR_Sangam_NoBuildings", "map_name": "상암 (건물 없음)"},
    5: {"folder_name": "R_KR_PR_SejongBRT0", "map_name": "세종 BRT"},
    6: {"folder_name": "R_US_PR_LVCC", "map_name": "라스베가스 컨벤션 센터"},
}

router = APIRouter(
    prefix="/maps",
    tags=["지도"],
)

# --- API 엔드포인트 ---

@router.get(
    "/{map_id}",
    response_class=FileResponse,
    summary="지도 GeoJSON 파일 조회",
    description="미리 변환 및 최적화된 도로 데이터를 GeoJSON 파일 형태로 반환합니다.",
    responses={
        200: {
            "content": {"application/json": {}},
            "description": "도로 데이터가 포함된 GeoJSON 파일.",
        },
        404: {
            "status_code": 404,
            "description": "해당 맵 또는 GeoJSON 파일을 찾을 수 없습니다.",
        },
    },
)
async def get_map_geojson_by_id(map_id: int):
    """
    /maps/{map_id} 경로로 GET 요청 시, 해당 ID에 맞는
    'merged_road_surfaces.geojson' 파일을 직접 반환합니다.
    """
    map_info = MAP_METADATA.get(map_id)
    if not map_info:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"ID '{map_id}'에 해당하는 맵을 찾을 수 없습니다.",
        )

    # 반환할 최종 GeoJSON 파일의 전체 경로를 구성합니다.
    geojson_file_path = MAP_DATA_BASE_PATH / map_info["folder_name"] / "merged_road_surfaces.geojson"
    print(geojson_file_path)
    if not geojson_file_path.is_file():
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"맵의 GeoJSON 파일을 찾을 수 없습니다 (Path: {geojson_file_path}). 데이터 사전 처리 과정이 필요합니다.",
        )

    # FileResponse를 사용하여 파일을 효율적으로 반환합니다.
    return FileResponse(path=geojson_file_path, media_type="application/json")

