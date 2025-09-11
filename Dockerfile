# Dockerfile 

FROM python:3.10-slim

# 작업 디렉토리를 /app으로 설정
WORKDIR /app

# Dockerfile과 같은 위치에 있는 requirements.txt를 복사
COPY requirements.txt .
# RUN pip install --no-cache-dir -r requirements.txt 
RUN pip install --no-cache-dir -r requirements.txt

# Dockerfile과 같은 위치에 있는 모든 파일을 복사
COPY . .
# uvicorn 실행 
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]