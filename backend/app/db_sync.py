# app/db_sync.py

import os
import json
from dotenv import load_dotenv
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# .env 파일에서 환경 변수 로드
load_dotenv()

# FastAPI의 비동기 URL과 동일한 DB 주소를 사용합니다.
DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_USERNAME = os.getenv("DB_USERNAME")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")
DB_SSL_CONFIG = os.getenv("DB_SSL_CONFIG") # .env 파일에서 SSL 설정을 문자열로 가져옵니다.

if not all([DB_HOST, DB_PORT, DB_USERNAME, DB_PASSWORD, DB_NAME]):
    raise ValueError("App Error: Missing database configuration in .env file.")

# Celery 워커는 동기 드라이버인 'pymysql'을 사용합니다.
DATABASE_URL = f"mysql+pymysql://{DB_USERNAME}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

# 💡 수정점: db.py의 SSL 설정 로직을 그대로 가져옵니다.
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

# 동기(synchronous) SQLAlchemy 엔진 생성 시 connect_args를 전달합니다.
engine = create_engine(
    DATABASE_URL,
    connect_args=connect_args
)

# 동기 세션을 생성하는 SessionMaker
SessionMaker = sessionmaker(autocommit=False, autoflush=False, bind=engine)