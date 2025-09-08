# Dockerfile

FROM python:3.10-slim
WORKDIR /app

# 파이썬이 /app 폴더를 최상위로 인식하게 만듭니다.
ENV PYTHONPATH=/app

COPY ./backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY ./backend /app

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]