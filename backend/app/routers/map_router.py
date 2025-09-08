import json
from pathlib import Path
from typing import Dict, Any
from datetime import datetime

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import JSONResponse
from app.schemas.map_schema import ErrorMessage

# 지도 데이터 파일이 저장된 기본 경로
MAP_DATA_BASE_PATH = Path("app/maps/")

MAP_METADATA = {
    1: {"folder_path": "R_KR_PG_K-City", "map_name": "K-City", "created_at": datetime(2025, 9, 1)},
    2: {"folder_path": "R_KR_PG_KATRI", "map_name": "KATRI", "created_at": datetime(2025, 9, 1)},
    3: {"folder_path": "R_KR_PG_KIAPI", "map_name": "KIAPI", "created_at": datetime(2025, 9, 1)},
    4: {"folder_path": "R_KR_PR_Sangam_NoBuildings", "map_name": "상암 (건물 없음)", "created_at": datetime(2025, 9, 1)},
    5: {"folder_path": "R_KR_PR_SejongBRT0", "map_name": "세종 BRT", "created_at": datetime(2025, 9, 1)},
    6: {"folder_path": "R_US_PR_LVCC", "map_name": "라스베가스 컨벤션 센터", "created_at": datetime(2025, 9, 1)},
}


router = APIRouter(
    prefix="/maps",
    tags=["지도"],
    responses={404: {"model": ErrorMessage, "description": "Map not found"}}
)

@router.get(
    "/{map_id}",
    responses={
        status.HTTP_200_OK: {
            "description": "지도 데이터 조회 성공. 응답 본문에 모든 지도 JSON 데이터가 포함됩니다.",
            "content": {
                "application/json": {
                    "example": {"message": "200 OK"}
                }
            }
        },
        status.HTTP_404_NOT_FOUND: {
            "model": ErrorMessage, "description": "Map not found"
        }
    },
    status_code=status.HTTP_200_OK,
    summary="지도 정보 및 데이터 조회",
    description="map_id에 해당하는 지도의 메타데이터와 관련된 모든 JSON 파일 데이터를 함께 반환합니다."
)
async def get_map_by_id(map_id: int):
    """
    지정된 map_id에 해당하는 지도 정보를 조회합니다.
    (DB를 사용하지 않고 서버 내부 파일 시스템에서 직접 조회)
    """
    # 1. (수정) DB 조회 대신 딕셔너리에서 메타데이터 조회
    map_info = MAP_METADATA.get(map_id)
    if not map_info:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="해당 맵이 존재하지 않음.",
        )

    # 2. 메타데이터의 폴더 경로로 실제 파일 위치 확인
    map_folder_path = MAP_DATA_BASE_PATH / map_info["folder_path"]
    if not map_folder_path.is_dir():
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Map data directory '{map_info['folder_path']}' not found on server for map_id: {map_id}",
        )
    
    # 3. 폴더 내 모든 JSON 파일을 읽어 딕셔너리로 조합
    combined_map_data: Dict[str, Any] = {}
    for file_path in map_folder_path.glob("*.json"):
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                file_key = file_path.stem 
                combined_map_data[file_key] = json.load(f)
        except (json.JSONDecodeError, FileNotFoundError) as e:
            print(f"Warning: Could not read or parse {file_path}. Error: {e}")
            continue

    # 4. 최종 응답 데이터 조합
    final_response_data = {
        "map_id": map_id,
        "map_name": map_info["map_name"],
        "created_at": map_info["created_at"].isoformat(),
        "map_data": combined_map_data,
    }

    # 5. JSONResponse를 직접 사용하여 실제 데이터를 응답 본문에 담아 반환
    return JSONResponse(content=final_response_data)

