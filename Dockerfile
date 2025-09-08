# Dockerfile

# 1. 베이스 이미지: 파이썬 3.10 슬림 버전을 사용합니다.
FROM python:3.10-slim

# 2. 작업 디렉토리: 컨테이너 내부에서 코드가 위치할 경로입니다.
WORKDIR /app

# 3. 의존성 설치: requirements.txt를 먼저 복사해서 설치합니다.
COPY ./backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 4. 소스코드 복사
COPY ./backend /app

# 5. 서버 실행: 컨테이너가 시작될 때 uvicorn 서버를 실행합니다.
#    컨테이너 내부에서는 8000번 포트를 사용합니다. (외부 접속은 Nginx가 담당)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]