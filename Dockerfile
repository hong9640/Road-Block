FROM python:3.10-slim

WORKDIR /app/backend
ENV PYTHONPATH=/app

# ✅ 리포 루트에 반드시 backend/requirements.txt 가 있어야 합니다.
COPY ./backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ✅ 리포 루트에 반드시 backend/app 디렉토리가 있어야 합니다.
COPY ./backend/app ./app

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
