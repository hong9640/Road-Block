# Road-Block: 도주 차량을 추적하는 자율주행 경찰차 시스템

## 프로젝트 개요
- Road-Block은 도주 차량을 실시간으로 식별·추적하고 자율주행 경찰차 군집을 투입해 안전하게 검거하는 통합 관제 시스템입니다.
- MORAI 시뮬레이터와 ROS를 통해 자율주행 추격 시나리오를 재현하고, FastAPI 기반 백엔드와 React 대시보드로 상황을 시각화합니다.
- 프로젝트 목표는 도주 차량 추적 과정에서 발생하는 2차 사고와 인명 피해를 최소화하고, 경찰관의 안전을 보장하는 것입니다.

## 시스템 구성
- 자율주행 시뮬레이터: MORAI + ROS Noetic에서 경찰차와 도주 차량의 주행 궤적과 센서 스트림을 생성합니다.
- Backend API: FastAPI + SQLModel로 차량/이벤트/지도 데이터를 관리하고 Redis·Celery로 비동기 작업을 처리합니다.
- Frontend Dashboard: React 19 + Vite 기반 관제 UI로 지도, 차량 상태, 이벤트 타임라인을 제공합니다.
- 인프라: Docker, Nginx, GitLab CI/CD, Private Registry로 빌드·배포 파이프라인을 구성합니다.

## 주요 기능
- 차량 위치 추적과 상태 모니터링(WebSocket + OpenLayers 지도)
- 검거/추적 이벤트 로깅 및 실시간 알림 스트림
- 지도 GeoJSON 다운로드와 MORAI 경로 데이터 연계
- ROS ↔ 백엔드 ↔ 대시보드 간 실시간 양방향 통신

## 기술 스택
- **Simulator**: MORAI Simulator, ROS1 Noetic, Ubuntu 20.04
- **Backend**: FastAPI, SQLModel(SQLAlchemy), MySQL, Celery, Redis, WebSocket
- **Frontend**: React 19, Vite, TypeScript, Zustand, OpenLayers, Tailwind CSS
- **Infra**: Docker, docker-compose, Nginx, GitLab CI, Private Registry

## 저장소 구조
```text
.
├─backend/
│ ├─app/
│ │ ├─common/
│ │ ├─maps/
│ │ ├─models/
│ │ ├─routers/
│ │ ├─schemas/
│ │ ├─services/
│ │ ├─celery_app.py
│ │ ├─db.py
│ │ ├─db_sync.py
│ │ ├─main.py
│ │ └─tasks.py
│ ├─Dockerfile
│ ├─requirements.txt
│ └─README.md
├─frontend/
│ ├─src/
│ └─Dockerfile
├─sim/
│ ├─simulator1/
│ │ └─catkin_ws/
│ │    ├─map_data/
│ │    └─src/
│ │      ├─morai_msgs/
│ │      └─road_block/
│ │         ├─launch/
│ │         └─scripts/
│ ├─simulator2/
│ │ └─catkin_ws/
│ │    └─src/
│ │      ├─morai_msgs/
│ │      └─road_block/
│ │         ├─launch/
│ │         └─scripts/
│ └─simulator3/
│    └─catkin_ws/
│       ├─map_data/
│       └─src/
│         ├─morai_msgs/
│         └─road_block/
│            ├─launch/
│            └─scripts/
├─docs/
│ ├─images/
│ └─ppt/
├─exec/
├─nginx/
├─docker-compose.yml
└─.gitlab-ci.yml
```
## 프로젝트 환경 설정

### 공통 사전 요구 사항
- Python 3.11, Node.js 20 이상, npm, Docker 24+ 및 Docker Compose Plugin
- MySQL 8.x 데이터베이스(로컬 또는 원격), Redis 7.x 이상
- ROS Noetic 및 MORAI Simulator가 설치된 Ubuntu 20.04 환경(시뮬레이터 연동 시)
- Git, OpenSSL, Certbot(HTTPS 인증서 자동화 사용 시)

### Backend 환경 준비(FastAPI)
1. 가상환경 생성 및 패키지 설치
   ```bash
   cd backend
   python -m venv .venv
   # Windows PowerShell
   .\.venv\Scripts\Activate
   # macOS/Linux
   # source .venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
2. `.env` 파일을 생성하고 다음 항목을 채웁니다.
   ```env
   DB_HOST=localhost
   DB_PORT=3306
   DB_USERNAME=roadblock
   DB_PASSWORD=roadblock_pw
   DB_NAME=roadblock
   DB_SSL_CONFIG={"ssl_true": true}
   HMAC_SECRET_KEY=replace_me
   REDIS_HOST=localhost
   ```
   > 주의: `DB_SSL_CONFIG` 값은 SSL 검증용 JSON이므로 삭제하거나 키 이름을 변경하지 마세요.
3. MySQL에 데이터베이스를 생성합니다.
   ```sql
   CREATE DATABASE roadblock CHARACTER SET utf8mb4;
   ```
4. FastAPI 애플리케이션이 최초 실행 시 테이블을 자동 생성하므로 별도의 마이그레이션 단계는 필요하지 않습니다.

### Frontend 환경 준비(React + Vite)
1. 패키지 설치
   ```bash
   cd frontend
   npm install
   ```
2. `frontend/.env` 파일을 작성해 API 및 WebSocket 엔드포인트를 지정합니다.
   ```env
   VITE_API_BASE=http://localhost:8000/api
   VITE_WS_BASE=ws://localhost:8000/ws
   ```

### ROS & 시뮬레이터 환경 준비(Ubuntu 20.04 기준)
1. Ubuntu 20.04 LTS 설치 후 필수 패키지 업데이트
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install build-essential curl git python3-pip python3-venv -y
   ```
2. ROS Noetic 설치 및 초기화
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt install curl -y
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo apt update
   sudo apt install ros-noetic-desktop-full ros-noetic-rosbridge-server -y
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
   sudo rosdep init
   rosdep update
   ```
3. 저장소를 Ubuntu 환경에 배치하고 catkin workspace 빌드
   ```bash
   git clone <REPO_URL>
   cd S13P21A507/sim/simulator3/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   source devel/setup.bash
   ```
   - 필요 시 `simulator1`, `simulator2` workspace도 동일한 방식으로 빌드할 수 있습니다.
4. MORAI Simulator 설치 및 환경 변수 설정
    - MORAI에서 제공한 지도 자산을 `sim/simulator3/catkin_ws/map_data` 및 `scripts/lib/mgeo_data`에 복사합니다.
5. ROS ↔ Backend 통신 준비
   ```bash
   echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
   echo "export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
   source ~/.bashrc
   ```
   - Redis 서버를 실행하고 백엔드 `.env`의 `REDIS_HOST`와 일치하도록 설정합니다.

### Docker 환경 준비(선택)
1. 로컬 Private Registry를 사용한다면 `docker login <registry>`로 인증합니다.
2. 배포용 `.env`를 `backend/.env`, `frontend/.env` 위치에 복사합니다.
3. GitLab Runner에는 Docker 빌드 권한과 Private Registry 접근 권한이 필요합니다.

## 프로젝트 실행

### 로컬 개발 모드
1. Redis와 MySQL을 기동합니다.
2. Backend API 서버 실행
   ```bash
   cd backend
   .\.venv\Scripts\Activate  # 또는 source .venv/bin/activate
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```
3. Celery Worker 실행(비동기 작업 사용 시)
   ```bash
   cd backend
   celery -A app.celery_app worker --loglevel=info
   ```
4. Frontend 개발 서버 실행
   ```bash
   cd frontend
   npm run dev -- --host --port 5173
   ```
   - 브라우저에서 `http://localhost:5173` 접속

### Docker 배포 및 실행 (EC2)

#### 주요 작업 방식 (Workflow)

이 프로젝트는 개발자가 로컬 PC에서 Docker를 실행하여 확인하는 과정이 없습니다. 로컬에서는 코드 작성 및 GitLab `push` 역할만 수행하며, 모든 빌드와 실행, 확인 작업은 GitLab CI/CD와 EC2 서버를 통해 이루어집니다.

#### 배포 절차

1.  로컬 PC에서 수정한 코드를 GitLab에 `push` 합니다.
2.  GitLab CI/CD 파이프라인이 완료되어 새 이미지가 생성될 때까지 기다립니다.
3.  EC2 서버에 접속하여, 아래 명령어를 순서대로 실행해 최신 버전으로 업데이트합니다.
    ```bash
    # 1. EC2 프라이빗 레지스트리에서 최신 이미지를 받아옵니다.
    docker-compose pull

    # 2. 기존 컨테이너를 내립니다. (프로젝트 환경의 포트 충돌 방지를 위해 필수)
    docker-compose down

    # 3. 새 이미지로 컨테이너를 실행합니다.
    docker-compose up -d
    ```
4.  배포된 서버의 주소로 접속하여 변경 사항이 정상적으로 반영되었는지 확인합니다.

#### 컨테이너 로그 확인

EC2 서버에서 실행 중인 컨테이너의 실시간 로그는 다음 명령어로 확인할 수 있습니다.
```bash
docker-compose logs -f backend
docker-compose logs -f frontend
```

### ROS 시뮬레이터 연동 절차
1. ROS 환경 초기화
   ```bash
   cd sim/simulator3/catkin_ws
   source devel/setup.bash
   ```
2. ROSBridge WebSocket 서버 실행
   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
3. 경찰차 시나리오 실행(터미널 별 실행)
   - `sim/simulator1/catkin_ws`: `roslaunch road_block police_car_1.launch`
   - `sim/simulator3/catkin_ws`: `roslaunch road_block police_car_1.launch`, `roslaunch road_block police_car_2.launch`
4. 도주 차량(러너) 등록
   ```bash
   cd sim/simulator2/catkin_ws
   source devel/setup.bash
   roslaunch road_block runner_car.launch
   ```
5. MORAI Simulator에서 대응 맵과 주행 시나리오를 실행합니다.
6. 백엔드 로그(`event_broadcasts.log`)와 프론트엔드 대시보드로 차량 위치·상태·이벤트 갱신을 확인합니다.

## 환경 변수
| 서비스 | 키 | 설명 |
|--------|----|------|
| Backend | `DB_HOST`, `DB_PORT`, `DB_USERNAME`, `DB_PASSWORD`, `DB_NAME` | MySQL 연결 정보 |
| Backend | `DB_SSL_CONFIG` | MySQL SSL 구성(JSON 문자열) |
| Backend | `HMAC_SECRET_KEY` | ROS 메시지 서명용 시크릿 |
| Backend | `REDIS_HOST` | Celery/Redis 호스트 |
| Frontend | `VITE_API_BASE` | REST API 기본 URL |
| Frontend | `VITE_WS_BASE` | WebSocket 기본 URL |

## 세부 디렉터리 가이드

### backend/
- `app/main.py`에서 FastAPI 앱과 CORS, GZip, lifespan 훅을 초기화합니다.
- `app/routers/`는 지도(`map_router.py`), 차량(`vehicle_router.py`), WebSocket(`websocket_router.py`) 엔드포인트를 제공합니다.
- `app/services/`는 차량 CRUD와 이벤트 조회, ROS 패킷 처리(`websocket_service.py`)를 담당합니다.
- `app/common/`에는 WebSocket 메시지 타입과 에러 코드를 정의한 `ws_codes.py`가 있습니다.
- `app/models/`는 SQLModel 기반 도메인 모델과 Enum(`models/enums.py`)을 제공합니다.
- `app/schemas/`는 REST 및 WebSocket 응답 스키마를 관리하며 `error_schema.py`에서 API 에러 코드를 정의합니다.
- `app/maps/`에는 최적화된 GeoJSON(`merged_road_surfaces.geojson`)이 맵 ID별로 저장됩니다.
- `Dockerfile`은 Python 3.11 slim 이미지를 기반으로 `requirements.txt`를 설치하고 `uvicorn`을 기동합니다.

### frontend/
- `src/pages/`는 메인, 지도, 로그 페이지 진입점을 제공합니다.
- `src/components/`는 `DashboardLayout.tsx`, `InfoEventModal.tsx` 등 공통 UI 컴포넌트를 포함합니다.
- `src/features/`는 도메인별 UI 로직(대시보드, 이벤트, 지도, 랜딩)을 분리합니다.
- `src/stores/`는 `useVehicleStore.ts`, `useEventStore.ts` 등 Zustand 상태 관리를 제공합니다.
- `src/websockets/`는 `WSProvider.tsx`, `handlers.ts`로 프론트엔드 WebSocket 연결과 메시지 파서를 구현합니다.
- `Apis.ts`는 Axios 인스턴스와 REST 요청 래퍼를 제공합니다.

### sim/
- `simulator1`은 `road_block` 패키지의 초기 구성과 기초 추격 스크립트를 포함합니다.
- `simulator2`는 `LaneFollowingPursuitNode.py` 등 변형 로직과 러너 등록 시나리오를 실험합니다.
- `simulator3`은 실제 Road-Block 시나리오, 메시지 정의(`morai_msgs/`), 지도 자산(`map_data/`)을 포함한 운영 workspace입니다.

### 문서 및 실행 자료
- `docs/images/`에는 시스템 구조도, 워크플로, ERD, 시나리오 스토리보드가 PNG로 정리되어 있습니다.
- `docs/ppt/`는 발표 슬라이드 원본을 보관합니다.
- `exec/`는 배포/운영 매뉴얼과 캡처 이미지를 제공합니다.

### 인프라 & 배포
- `nginx/default.conf`는 HTTP→HTTPS 리다이렉트, `/api` 및 `/ws` 프록시, Certbot 챌린지 경로를 설정합니다.
- `.gitlab-ci.yml`은 백엔드·프론트엔드 이미지를 빌드해 `localhost:5000` 레지스트리에 푸시하는 파이프라인을 정의합니다.
- `docker-compose.yml`은 Redis, 백엔드, Celery Worker, 프론트엔드, Nginx를 단일 브리지 네트워크로 구성합니다.

## 통신 규약

### REST API (주요)
| Method | Endpoint | 설명 |
|--------|----------|------|
| GET | `/api/` | 헬스 체크 메시지 |
| GET | `/api/maps/{map_id}` | 맵 ID별 GeoJSON 다운로드 |
| GET | `/api/vehicles` | 등록 차량 목록 조회 |
| GET | `/api/vehicles/{id}` | 단일 차량 상세 |
| PATCH | `/api/vehicles/{id}` | 차량 이름 수정 |
| DELETE | `/api/vehicles/{id}` | 차량 삭제(논리 삭제) |
| GET | `/api/vehicles/events` | 차량 이벤트 로그 |

### WebSocket 채널
| 경로 | 발신 주체 | 수신 주체 | 메시지 |
|------|-----------|-----------|--------|
| `/ws/vehicles` | ROS 노드 | Backend ↔ ROS | 차량 등록, 위치 브로드캐스트, 상태 갱신 |
| `/ws/events` | ROS 노드 | Backend ↔ ROS | 검거 이벤트, 실패 이벤트 |
| `/ws/front/vehicles` | Backend | Frontend | 차량 위치·상태 스트림 |
| `/ws/front/events` | Backend | Frontend | 추적/검거 이벤트 알림 |

> WebSocket 메시지 타입은 `backend/app/common/ws_codes.py`에 정의되어 있으며, `websocket_service.py`가 패킷 파싱과 브로드캐스트를 담당합니다.

## 시뮬레이터 시나리오 요약
- `0_coordinate.py`~`5_gear.py`: 좌표계 설정, 직진, 추격, 후진 등 단계별 주행 스크립트
- `chase_manager.py`, `parameterized_pursuit_node.py`: 다수 경찰차의 포위 제어 파라미터 조정
- `vehicle_registration.py`, `vehicle_stat_info.py`: ROS 메시지를 백엔드로 전달해 등록/상태 업데이트 수행
- `manual_chase_commander.py`: 수동 제어와 자동 추격 전환 테스트

## 참고 문서
- [기능 명세서](https://www.notion.so/262546b4de0280d3a988ea11e464ff57)
- [API 명세서](https://magnificent-lighter-489.notion.site/API-262546b4de0280779251d4816ce29941?pvs=74)
- [Figma Lo-Fi 와이어프레임](https://www.figma.com/design/654U42E22pFG5uiDIWTAFy/%ED%8A%B9%ED%99%94-%ED%8E%98%EC%9D%B4%EC%A7%80-%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83-%EC%B4%88%EC%95%88?node-id=0-1)

