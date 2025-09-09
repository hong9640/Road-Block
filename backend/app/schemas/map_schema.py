from datetime import datetime
from typing import List
from pydantic import BaseModel

class MapInfoResponse(BaseModel):
    """
    지도 메타데이터와 파일 URL 목록을 포함하는 응답 스키마
    """
    map_id: int
    map_name: str
    created_at: datetime
    file_urls: List[str] # 실제 데이터 대신 URL 리스트를 포함

    class Config:
        # FastAPI 문서에 표시될 예시 데이터
        schema_extra = {
            "example": {
                "map_id": 1,
                "map_name": "K-City",
                "created_at": "2025-09-09T12:00:00Z",
                "file_urls": [
                    "/static/maps/R_KR_PG_K-City/crosswalk_set.json",
                    "/static/maps/R_KR_PG_K-City/link_set.json"
                ]
            }
        }