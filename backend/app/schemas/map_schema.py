from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field

class ErrorMessage(BaseModel):
    """예외(오류) 메시지를 위한 스키마"""
    message: str = Field(..., example="해당 맵이 존재하지 않음.")

class MapResponse(BaseModel):
    """/maps/{map_id} API의 최종 응답 스키마"""
    map_id: int
    map_name: str
    created_at: datetime
    
    # map_data는 파일 이름을 key로, 파일 내용을 value로 갖는 딕셔너리입니다.
    map_data: Dict[str, Any]

    class Config:
        # 예시 데이터 자동 생성을 위한 설정
        schema_extra = {
            "example": {
                "map_id": 1,
                "map_name": "강남대로",
                "created_at": "2025-09-05T15:00:00Z",
                "map_data": {
                    "crosswalk_set": [{ "id": 1, "points": [...] }],
                    "link_set": [{ "id": "A21", "lanes": [...] }]
                }
            }
        }
