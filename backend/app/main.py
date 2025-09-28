# app/main.py

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from dotenv import load_dotenv
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from app.schemas.error_schema import ErrorMessage, APIException
from fastapi.middleware.gzip import GZipMiddleware

# .env 파일 로드
load_dotenv()

# 이제 다른 모듈들을 상대 경로로 안전하게 임포트합니다.
from app.db import create_db_and_tables
from app.routers import map_router, vehicle_router, websocket_router

# Lifespan 컨텍스트 매니저 정의
# 애플리케이션 시작과 종료 시 처리할 로직
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("--- FastAPI app startup: creating DB tables... ---")
    await create_db_and_tables()
    yield
    print("--- FastAPI app shutdown. ---")

# FastAPI 앱 인스턴스 생성
app = FastAPI(lifespan=lifespan, root_path="/api")

# --- 전역 예외 핸들러 --- 
@app.exception_handler(APIException)
async def api_exception_handler(request: Request, exc: APIException):
    return JSONResponse(
        status_code=exc.status_code,
        content=ErrorMessage(message=exc.message).model_dump(),
    )
# 예상치 못한 모든 예외를 처리하여 500 응답을 반환
@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
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
    allow_origins=["*"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.add_middleware(GZipMiddleware, minimum_size=1000)

# --- 라우터 등록 --- 
app.include_router(map_router.router)
app.include_router(vehicle_router.router)
app.include_router(websocket_router.router)

# 기본 루트 엔드포인트
@app.get("/")
def read_root():
    return {"message": "Server is running successfully!"}
