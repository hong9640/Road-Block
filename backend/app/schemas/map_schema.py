from datetime import datetime
from pydantic import BaseModel, Field

class MapInfoResponse(BaseModel):
    """
    지도 메타데이터와 최적화된 GeoJSON 파일의 URL을 포함하는 응답 스키마
    """
    map_id: int
    map_name: str
    created_at: datetime
    # 최종적으로 프론트엔드가 사용할 GeoJSON 파일의 URL 하나만 제공합니다.
    geojson_url: str = Field(..., description="도로 데이터를 담고 있는 최적화된 GeoJSON 파일의 경로")

    class Config:
        # FastAPI 문서에 표시될 예시 데이터
        schema_extra = {
            "example": {
                "map_id": 1,
                "map_name": "K-City",
                "created_at": "2025-09-09T12:00:00Z",
                # 프론트엔드가 이 URL로 직접 GET 요청을 보낼 수 있습니다.
                "geojson_url": "/maps/1/geojson"
            }
        }
