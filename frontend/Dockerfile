# 1. 빌드 스테이지: Node.js 기반 환경에서 React 앱을 빌드합니다.
FROM node:18-alpine AS builder

# 컨테이너 내 작업 디렉토리 설정
WORKDIR /app

# 의존성 파일만 먼저 복사하여 캐시를 활용
COPY frontend/package.json frontend/package-lock.json ./
RUN npm install

# 소스 코드 전체를 복사
COPY . .

# React 애플리케이션 빌드
RUN npm run build

# 2. 서빙 스테이지: Nginx 기반 환경에서 빌드된 정적 파일을 서빙합니다.
FROM nginx:1.23-alpine

# 빌드 스테이지에서 생성된 빌드 결과물을 Nginx 정적 파일 위치로 복사
COPY --from=builder /app/build /usr/share/nginx/html

# Nginx가 기본적으로 80번 포트를 사용하므로 별도의 EXPOSE 8080은 불필요.
# Nginx 실행
CMD ["nginx", "-g", "daemon off;"]