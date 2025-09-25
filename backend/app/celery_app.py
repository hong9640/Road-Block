# app/celery_app.py (신규 파일)

from celery import Celery

# Redis 서버 주소
REDIS_URL = "redis://localhost:6379/0"

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