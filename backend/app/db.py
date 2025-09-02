# app/db.py

import os
import json
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlmodel import SQLModel


DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_USERNAME = os.getenv("DB_USERNAME")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")
DB_SSL_CONFIG = os.getenv("DB_SSL_CONFIG") # .env 파일에서 SSL 설정을 문자열로 가져옵니다.

if not all([DB_HOST, DB_PORT, DB_USERNAME, DB_PASSWORD, DB_NAME]):
    raise ValueError("App Error: Missing database configuration in .env file.")

DATABASE_URL = f"mysql+asyncmy://{DB_USERNAME}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

# SSL 설정을 처리합니다.
connect_args = {}
if DB_SSL_CONFIG:
    try:
        # DB_SSL_CONFIG 환경 변수(문자열)를 JSON으로 파싱하여 connect_args에 추가합니다.
        ssl_config = json.loads(DB_SSL_CONFIG)
        connect_args["ssl"] = ssl_config
    except json.JSONDecodeError:
        # 단순 "true" 문자열일 경우 True 불리언으로 변환합니다.
        if DB_SSL_CONFIG.lower() == "true":
            connect_args["ssl"] = True
        else:
            # 유효하지 않은 형식의 값일 경우 에러를 발생시킵니다.
            raise ValueError(f"Invalid DB_SSL_CONFIG format: {DB_SSL_CONFIG}")

engine = create_async_engine(
    DATABASE_URL,
    echo=True,
    connect_args=connect_args
)

# SessionMaker를 모듈 레벨에서 한 번만 생성합니다.
AsyncSessionMaker = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


async def get_session() -> AsyncGenerator[AsyncSession, None]:
    """FastAPI 의존성 주입을 위한 비동기 데이터베이스 세션 생성기"""
    async with AsyncSessionMaker() as session:
        yield session


async def create_db_and_tables():
    """SQLModel 메타데이터를 기반으로 모든 테이블을 비동기적으로 생성합니다."""
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)
    print("--- Database tables created successfully. ---")
