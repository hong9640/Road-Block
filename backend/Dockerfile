# 1. 베이스 이미지: 파이썬 3.11 슬림 버전
FROM python:3.11-slim

# 2. 컨테이너 내 작업 디렉토리 설정
WORKDIR /code

# 3. 의존성 파일만 먼저 복사하여 Docker 빌드 캐시 활용
COPY backend/requirements.txt .

# 4. 의존성 설치
RUN pip install --no-cache-dir -r requirements.txt

# 5. backend 폴더 안의 모든 소스 코드를 컨테이너의 /code 폴더로 복사
COPY backend .

# 6. FastAPI 앱이 실행될 8000번 포트 노출
EXPOSE 8000

# 7. 애플리케이션 실행
# WORKDIR가 /code 이므로, 그 안의 app 폴더를 인식하여 app.main 을 실행합니다.
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]