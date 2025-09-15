# models/error.py (파일을 분리하는 것을 추천)

from pydantic import BaseModel

class ErrorMessage(BaseModel):
    message: str

class APIException(Exception):
    def __init__(self, status_code: int, message: str):
        self.status_code = status_code
        self.message = message

class InvalidBodyException(APIException):
    def __init__(self):
        super().__init__(status_code=400, message="status값이 Enum에 존재하지 않는 등 Body 데이터가 유효하지 않음.")

class MapNotFoundException(APIException):
    def __init__(self):
        super().__init__(status_code=404, message="해당 맵이 존재하지 않음.")
        
class InternalServerErrorException(APIException):
    def __init__(self):
        super().__init__(status_code=500, message="서버 내부 오류")