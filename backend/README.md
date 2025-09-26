# 경찰차 자율주행 로드블락 시스템 - Backend

## 🛠 기술 스택

- **Framework**: FastAPI
- **Database**: MySQL (with SQLModel/SQLAlchemy)
- **Task Queue**: Celery
- **WebSocket**: FastAPI WebSocket
- **Other Libraries**:
  - uvicorn: ASGI 서버
  - pydantic: 데이터 검증
  - python-dotenv: 환경변수 관리
  - asyncmy: 비동기 MySQL 드라이버
  - hmac/hashlib: 메시지 인증

## ⚙️ 환경 변수 설정

`.env` 파일에 다음 환경변수들을 설정해야 합니다:

```
DB_HOST=localhost
DB_PORT=3306
DB_USERNAME=user
DB_PASSWORD=password
DB_NAME=dbname
DB_SSL_CONFIG={"ssl_ca": "/path/to/ca.pem"} # 선택사항
HMAC_SECRET_KEY=your_secret_key
```

추가로, 임베디드와의 원활한 통신을 위해 다음을 추가로 설정합니다.

```
REDIS_HOST=redis
```

## 🚀 서버 실행

1. 의존성 설치
```bash
pip install -r requirements.txt
```

2. 서버 실행
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## 🗂 프로젝트 구조

```
backend/
├── app/ # 메인 애플리케이션 패키지
│   ├── __init__.py
│   ├── main.py           # FastAPI 앱 초기화 및 설정
│   ├── celery_app.py     # Celery 워커 설정
│   ├── db.py             # 데이터베이스 연결 및 기본 설정
│   ├── db_sync.py        # 동기 DB 연결용 
│   ├── tasks.py          # Celery 비동기 작업 정의
│   │
│   ├── common/           # 공통 모듈
│   │   ├── __init__.py
│   │   └── ws_codes.py   # WebSocket 메시지 타입 정의
│   │
│   ├── models/           # DB 모델 정의
│   │   ├── enums.py              # Enum 타입 정의
│   │   └── models.py             # SQLModel 기반 DB 모델
│   │
│   ├── routers/          # API 라우터
│   │   ├── map_router.py         # 지도 관련 API
│   │   ├── vehicle_router.py     # 차량 관련 API  
│   │   └── websocket_router.py   # WebSocket 연결 처리
│   │
│   ├── schemas/          # Pydantic 스키마
│   │   ├── error_schema.py       # 에러 응답 스키마
│   │   ├── vehicle_schema.py     # 차량 관련 스키마
│   │   └── websocket_schema.py   # WebSocket 메시지 스키마
│   │
│   └── services/         # 비즈니스 로직
│       ├── vehicle_service.py    # 차량 관련 서비스
│       └── websocket_service.py  # WebSocket 메시지 처리
│
├── maps/ # 지도 데이터 파일
├── Dockerfile # 도커 이미지 설정
└── requirements.txt # 의존성 목록
```

## 🔌 WebSocket Protocol

WebSocket 통신은 두 가지 주요 채널로 구분됩니다:

1. 웹 대시보드 통신 채널 (`/ws/front/*`)

| 채널 | 송신 | 수신 |
|------|------|------|
| `/ws/front/vehicles` | 차량 위치 실시간 업데이트 | 차량 상태 변경 요청 |
| `/ws/front/events` | 실시간 이벤트 알림 (검거/추적 상태) | 이벤트 처리 결과 확인 |





2. 임베디드 통신 채널 (`/ws/*`)

| 채널 | 송신 | 수신 |
|------|------|------|
| `/ws/vehicles` | - 위치 데이터 (`POSITION_BROADCAST`)<br>- 차량 등록 요청 (`REGISTER_REQUEST`)<br>- 상태 업데이트 (`STATUS_UPDATE_REQUEST`) | - 제어 명령<br>- 로드블락 배치 정보<br>- 등록 응답 |
| `/ws/events` | - 검거 이벤트 (`EVENT_CATCH`)<br>- 검거 실패 이벤트 (`EVENT_CATCH_FAILED`) | - 이벤트 확인 응답<br>- 추적 명령 |


모든 메시지 타입은 `app/common/ws_codes.py`에 정의되어 있습니다.

## 🚗 API Endpoints

| Method | Endpoint                | 설명               |
|--------|-------------------------|--------------------|
| GET    | `/maps`                 | 사용 가능한 지도 목록 조회 |
| GET    | `/vehicles`             | 등록된 차량 목록 조회     |
| GET    | `/vehicles/{id}`        | 특정 차량 정보 조회       |
| PATCH  | `/vehicles/{id}`        | 특정 차량 정보 수정       |
| GET    | `/vehicles/events`      | 차량 이벤트 로그 조회     |

각 API의 상세 스펙은 실행 후 `/docs`에서 확인할 수 있습니다.