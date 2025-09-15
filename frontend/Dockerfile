# =========================================
# Stage 1: Build the React.js Application
# =========================================
ARG NODE_VERSION=22.17.0
ARG NGINX_VERSION=alpine3.22

# Use a lightweight Node.js image for building (customizable via ARG)
FROM node:${NODE_VERSION}-alpine AS builder

# 빌드 환경 변수 정의 (선택사항)
# ARG REACT_APP_API_URL

# Set the working directory inside the container
WORKDIR /app

# 의존성 파일만 먼저 복사하여 캐시 활용
COPY package.json package-lock.json ./

# Install project dependencies using npm ci (ensures a clean, reproducible install)
RUN --mount=type=cache,target=/root/.npm npm ci

# 소스 코드 복사
COPY . .

# React 애플리케이션 빌드
# --build-arg로 전달된 환경 변수 사용
RUN npm run build

# =========================================
# Stage 2: Prepare Nginx to Serve Static Files
# =========================================

# ---- Runtime (Unprivileged NGINX, 기본 listen 8080) ----
FROM nginxinc/nginx-unprivileged:stable-alpine

# Copy custom Nginx config
COPY nginx.conf /etc/nginx/nginx.conf

# Copy the static build output from the build stage to Nginx's default HTML serving directory
COPY --from=builder /app/dist /usr/share/nginx/html

# Expose port 8080 to allow HTTP traffic
EXPOSE 8080

# Start Nginx directly with custom config
# Nginx 실행
ENTRYPOINT ["nginx", "-c", "/etc/nginx/nginx.conf"]
CMD ["nginx", "-g", "daemon off;"]
