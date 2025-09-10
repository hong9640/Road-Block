# Dockerfile

FROM python:3.10-slim
# --- (수정) WORKDIR을 /app/backend로 변경 ---
WORKDIR /app/backend

# 파이썬이 /app 폴더를 최상위로 인식하게 만듭니다.
ENV PYTHONPATH=/app

# --- (수정) 경로를 현재 WORKDIR 기준으로 단순화 ---
COPY ./backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY ./backend/app ./app

# --- (수정) WORKDIR이 /app/backend이므로, app.main을 바로 찾을 수 있음 ---
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
