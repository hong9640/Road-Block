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
from fastapi.responses import JSONResponse
from dotenv import load_dotenv
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware

# .env 파일 로드
load_dotenv()

# 이제 다른 모듈들을 상대 경로로 안전하게 임포트합니다.
from app.db import create_db_and_tables
from app.models import models



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

# --- 전역 예외 핸들러 --- 

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

# --- 라우터 등록 --- 

# --- WebSocket 엔드포인트 ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    Frontend 클라이언트와의 WebSocket 연결을 처리합니다.
    """
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            await websocket.send_text(f"Message text was: {data}")
    except WebSocketDisconnect:
        print("클라이언트와 연결이 끊어졌습니다.")


# 기본 루트 엔드포인트
@app.get("/")
def read_root():
    return {"message": "Server is running successfully!"}

# # 이 파일을 직접 실행할 때를 위한 코드
# if __name__ == "__main__":
#     uvicorn.run("app.main:app", host="0.0.0.0", port=int(os.getenv("MAIN_SERV_PORT")), reload=True)