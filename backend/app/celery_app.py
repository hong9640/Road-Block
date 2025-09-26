import os
from celery import Celery
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# Redis 서버 주소
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_URL = f"redis://{REDIS_HOST}:6379/0"

# Celery 앱 인스턴스 생성
celery_app = Celery(
    "worker",
    broker=REDIS_URL,
    backend=REDIS_URL,
    include=['app.tasks'] # Celery가 tasks.py 파일을 읽도록 설정
)

celery_app.conf.update(
    timezone='Asia/Seoul',
    enable_utc=False,
)