import io
import zipfile
from pathlib import Path
from datetime import datetime

from fastapi import APIRouter
from fastapi.responses import StreamingResponse

# 프로젝트 구조에 맞게 import 경로를 확인해주세요.
from app.schemas.error_schema import ErrorMessage, MapNotFoundException

# --- 상수 및 라우터 설정 ---

# 'app/maps'에 지도 데이터가 있는 것으로 가정합니다.
MAP_DATA_BASE_PATH = Path("app/maps/")

# DB를 대신하는 메타데이터
MAP_METADATA = {
    1: {"folder_path": "R_KR_PG_K-City", "map_name": "K-City", "created_at": datetime(2025, 9, 1)},
    2: {"folder_path": "R_KR_PG_KATRI", "map_name": "KATRI", "created_at": datetime(2025, 9, 1)},
    3: {"folder_path": "R_KR_PG_KIAPI", "map_name": "KIAPI", "created_at": datetime(2025, 9, 1)},
    4: {"folder_path": "R_KR_PR_Sangam_NoBuildings", "map_name": "상암 (건물 없음)", "created_at": datetime(2025, 9, 1)},
    5: {"folder_path": "R_KR_PR_SejongBRT0", "map_name": "세종 BRT", "created_at": datetime(2025, 9, 1)},
    6: {"folder_path": "R_US_PR_LVCC", "map_name": "라스베가스 컨벤션 센터", "created_at": datetime(2025, 9, 1)},
}

# main.py에서 'app.include_router(map_router.router)'로 등록되었으므로
# prefix를 여기에 정의합니다.
router = APIRouter(
    prefix="/maps",
    tags=["지도"],
)

# --- API 엔드포인트 ---

@router.get(
    "/{map_id}",
    summary="지도 데이터 압축 파일 다운로드",
    description="해당 맵의 모든 JSON 데이터를 하나의 ZIP 파일로 압축하여 반환합니다.",
    responses={
        200: {
            "content": {"application/zip": {}},
            "description": "지도 데이터가 포함된 ZIP 파일.",
        },
        404: {
            "model": ErrorMessage,
            "description": "해당 맵을 찾을 수 없습니다.",
        },
    },
)
async def download_map_zip_by_id(map_id: int):
    """
    /maps/{map_id} 경로로 요청 시, 모든 관련 지도 JSON 파일을
    실시간으로 압축하여 하나의 ZIP 파일로 다운로드합니다.
    """
    map_info = MAP_METADATA.get(map_id)
    if not map_info:
        raise MapNotFoundException()

    map_folder_path = MAP_DATA_BASE_PATH / map_info["folder_path"]
    if not map_folder_path.is_dir():
        raise MapNotFoundException()

    zip_buffer = io.BytesIO()

    with zipfile.ZipFile(zip_buffer, "w", zipfile.ZIP_DEFLATED) as zf:
        for file_path in map_folder_path.glob("*.json"):
            zf.write(file_path, arcname=file_path.name)
    
    zip_buffer.seek(0)

    zip_filename = f"{map_info['folder_path']}.zip"
    
    headers = {
        'Content-Disposition': f'attachment; filename="{zip_filename}"'
    }

    return StreamingResponse(
        zip_buffer,
        media_type="application/zip",
        headers=headers
    )
