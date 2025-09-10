# Dockerfile

FROM python:3.10-slim

# 작업 디렉토리를 /app으로 단순화
WORKDIR /app

COPY ./backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 나머지 백엔드 폴더 전체를 복사
COPY ./backend .

# uvicorn 실행
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]