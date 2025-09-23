# app/main.py
# app 폴더의 메인 실행 파일

# 프로젝트 루트 디렉토리를 Python 경로에 추가하여 모듈 임포트 문제 해결
import sys
import os
# importerror 해결
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import uvicorn
from datetime import datetime
from fastapi import FastAPI, Request, status, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse,FileResponse
from dotenv import load_dotenv
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from app.schemas.error_schema import ErrorMessage, APIException
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.staticfiles import StaticFiles

# .env 파일 로드
load_dotenv()

# 이제 다른 모듈들을 상대 경로로 안전하게 임포트합니다.
from app.db import create_db_and_tables
from app.models import models
from app.routers import map_router, vehicle_router, websocket_router




# Lifespan 컨텍스트 매니저 정의
@asynccontextmanager
async def lifespan(app: FastAPI):
    """애플리케이션 시작과 종료 시 처리할 로직"""
    print("--- FastAPI app startup: creating DB tables... ---")
    await create_db_and_tables()
    yield
    print("--- FastAPI app shutdown. ---")

# FastAPI 앱 인스턴스 생성
app = FastAPI(lifespan=lifespan)
app = FastAPI(
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json"
)

# --- 전역 예외 핸들러 --- 
@app.exception_handler(APIException)
async def api_exception_handler(request: Request, exc: APIException):
    """커스텀 APIException을 처리하여 JSON 응답을 반환합니다."""
    return JSONResponse(
        status_code=exc.status_code,
        content=ErrorMessage(message=exc.message).model_dump(),
    )

@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    """예상치 못한 모든 예외를 처리하여 500 응답을 반환합니다."""
    # 프로덕션 환경에서는 에러 로깅(logging)을 추가해야 합니다.
    print(f"An unexpected error occurred: {exc}")
    return JSONResponse(
        status_code=500,
        content=ErrorMessage(message="서버 내부 오류").model_dump(),
    )

# --- 미들웨어 설정 --- 

# CORS 미들웨어 설정
origins = [
    "http://localhost",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 운영 환경에서는 origins 변수 사용 권장
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.add_middleware(GZipMiddleware, minimum_size=1000)

# app.mount("/static", StaticFiles(directory="static"), name="static")
# --- 라우터 등록 --- 
app.include_router(map_router.router)
app.include_router(vehicle_router.router)
app.include_router(websocket_router.router)

# 시험용
# @app.get("/", include_in_schema=False)
# async def read_index():
#     """웹 브라우저에서 접속 시 테스트용 index.html 파일을 반환합니다."""
#     return FileResponse('static/index.html')

# 기본 루트 엔드포인트
@app.get("/")
def read_root():
    return {"message": "Server is running successfully!"}

# # 이 파일을 직접 실행할 때를 위한 코드
# if __name__ == "__main__":
#     uvicorn.run("app.main:app", host="0.0.0.0", port=int(os.getenv("MAIN_SERV_PORT")), reload=True)