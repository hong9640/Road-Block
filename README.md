# Road-Block: 도주 차량을 추적하는 자율주행 경찰차 시스템

## 프로젝트 개요
- Road-Block은 도주 차량을 실시간으로 식별·추적하고 자율주행 경찰차 군집을 투입해 안전하게 검거하는 통합 관제 시스템입니다.
- MORAI 시뮬레이터와 ROS를 통해 자율주행 시나리오를 재현하고, FastAPI 기반 백엔드와 React 대시보드로 상황을 시각화합니다.
- 프로젝트 목표는 도주 차량 추적 과정에서 발생하는 2차 사고와 인명 피해를 최소화하고, 경찰관의 안전을 보장하는 것입니다.

## 시스템 구성
- 자율주행 시뮬레이터: MORAI + ROS Noetic에서 여러 경찰 차량과 도주 차량의 주행 궤적을 생성합니다.
- Backend API: FastAPI + SQLModel로 차량, 이벤트, 지도 데이터를 관리하고 Redis·Celery로 비동기 작업을 처리합니다.
- Frontend Dashboard: React 19 + Vite 기반 관제 인터페이스로 지도, 차량 상태, 이벤트 타임라인을 제공합니다.
- 인프라: Docker, Nginx, GitLab CI/CD, Private Registry로 빌드·배포 파이프라인을 구성합니다.

## 주요 기능
- 차량 위치 추적과 상태 모니터링(WebSocket + OpenLayers 지도)
- 검거/추적 이벤트 로깅 및 알림 스트림
- 지도 GeoJSON 다운로드와 MORAI 경로 데이터 연계
- 차량 메타데이터 CRUD 및 더미 데이터 생성 스크립트
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
│ ├─maps/
│ ├─create_dummy_*.py
│ └─Dockerfile
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
│            ├─msg/
│            ├─scripts/
│            └─srv/
├─docs/
│ ├─images/
│ └─ppt/
├─exec/
├─nginx/
├─docker-compose.yml
└─.gitlab-ci.yml
```
- `backend/` FastAPI 서비스와 Celery 작업, 지도 GeoJSON 데이터, 더미 데이터 스크립트가 위치합니다.
- `frontend/`는 관제 대시보드 소스와 빌드/배포를 위한 Vite, Tailwind 설정을 포함합니다.
- `sim/`에는 단계별 catkin workspace와 ROS 패키지, MORAI 지도 자산(`mgeo`)이 정리되어 있습니다.
- `docs/`는 발표 자료, 구조도, ERD 이미지와 PPT 파일을 보관합니다.
- `exec/`는 운영·시연 매뉴얼과 캡처 이미지를 제공합니다.
- `nginx/`는 HTTPS 리버스 프록시 설정과 Certbot 챌린지 경로를 정의합니다.
- 루트의 `docker-compose.yml`, `.gitlab-ci.yml`은 배포 파이프라인과 이미지 빌드 규칙을 설명합니다.

## 세부 디렉토리 가이드

### backend/
- `app/main.py`에서 FastAPI 앱과 CORS, GZip, lifespan 훅을 초기화합니다.
- `app/routers/`는 지도(`map_router.py`), 차량(`vehicle_router.py`), WebSocket(`websocket_router.py`) 엔드포인트를 제공합니다.
- `app/services/`는 차량 CRUD와 이벤트 조회 로직, ROS 패킷 처리(`websocket_service.py`)를 구현합니다.
- `app/models/models.py`는 SQLModel 기반 차량·이벤트·경찰차 모델과 Enum(`models/enums.py`)을 정의합니다.
- `app/schemas/`는 REST 및 WebSocket 응답 스키마를 관리하며 예외 처리용 `error_schema.py`를 포함합니다.
- `app/maps/`에는 최적화된 GeoJSON(`merged_road_surfaces.geojson`) 세트가 맵 ID별로 저장돼 있습니다.
- 루트의 `create_dummy_*.py` 스크립트는 테스트용 차량, 이벤트, 상태 데이터를 생성합니다.
- `Dockerfile`은 Python 3.11 slim 이미지에 `requirements.txt`를 설치하고 `uvicorn`을 기동합니다.

### frontend/
- `src/pages/`는 메인, 지도, 로그 페이지 진입점을 제공합니다.
- `src/components/`의 `DashboardLayout.tsx`, `InfoEventModal.tsx` 등이 공통 UI 블록을 구성합니다.
- `src/features/`는 도메인별 UI 로직(대시보드, 이벤트, 맵, 랜딩)을 분리합니다.
- `src/stores/`의 `useVehicleStore.ts`, `useEventStore.ts`는 Zustand 기반 상태 관리를 담당합니다.
- `src/websockets/`의 `WSProvider.tsx`, `handlers.ts`는 사용자 단 WebSocket 연결·파서를 제공합니다.
- `Apis.ts`는 Axios 인스턴스와 REST 요청 래퍼를 정의하며, `lib/datas.ts`는 더미 데이터·상수 모음을 제공합니다.
- `Dockerfile`은 Node 20-alpine으로 빌드한 후 Nginx 1.23-alpine에서 정적 파일을 서비스합니다.

### sim/
- `simulator*`은 `road_block/` 패키지의 초기 구성을 담은 워크스페이스로, 기본 추격 시나리오 스크립트를 포함합니다.
- `simulator2`는 `runner_car.launch` 등 통신과 메인 노드를 포함합니다.

### 문서 및 실행 자료
- `docs/images/`에는 시스템 구조도, 워크플로, ERD, 시나리오 스토리보드가 PNG 포맷으로 정리돼 있습니다.
- `docs/ppt/`는 발표용 슬라이드 원본을 보관합니다.
- `exec/`의 운영 매뉴얼은 GitLab 빌드·배포 절차와 ROS 기반 추적 시나리오 실행법을 서술합니다.

### 인프라 & 배포
- `nginx/default.conf`는 HTTP→HTTPS 리다이렉트, `/api` 및 `/ws` 프록시, 정적 자산 프록시를 설정하며 Certbot 경로를 노출합니다.
- `.gitlab-ci.yml`은 백엔드·프론트엔드 이미지를 빌드하고 `localhost:5000` 레지스트리에 푸시하는 `push` 스테이지를 정의합니다.
- `docker-compose.yml`은 서비스별 컨테이너 이름과 의존성을 설정하고, Nginx가 프론트엔드/백엔드로 요청을 라우팅합니다.

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
   . .venv/Scripts/Activate.ps1
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
   DB_SSL_CONFIG={"ssl": false}  # SSL 미사용 시 생략 가능
   HMAC_SECRET_KEY=replace_me
   REDIS_HOST=localhost
   ```
3. MySQL에 데이터베이스를 생성합니다.
   ```sql
   CREATE DATABASE roadblock CHARACTER SET utf8mb4;
   ```
4. 최초 실행 시 FastAPI 애플리케이션이 자동으로 테이블을 생성하므로 별도의 마이그레이션 단계는 필요하지 않습니다.

### Frontend 환경 준비(React + Vite)
1. 패키지 설치 및 환경 변수 파일 작성
   ```bash
   cd frontend
   npm install
   ```
2. `frontend/.env` 파일을 생성하여 API 및 WebSocket 엔드포인트를 지정합니다.
   ```env
   VITE_API_BASE=http://localhost:8000/api
   VITE_WS_BASE=ws://localhost:8000/ws
   ```

### ROS & 시뮬레이터 환경 준비(Ubuntu 20.04 기준)
1. Ubuntu 20.04 LTS 설치
   - Bare metal PC 또는 Windows 11/10의 WSL2에 Ubuntu 20.04를 설치합니다.
   - 설치 직후 필수 패키지를 업데이트합니다.
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install build-essential curl git python3-pip python3-venv -y
   ```
2. ROS Noetic 설치 및 기본 설정
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt install curl -y
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo apt update
   sudo apt install ros-noetic-desktop-full -y
   sudo apt install ros-noetic-rosbridge-server -y
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
   sudo rosdep init
   rosdep update
   ```
3. 저장소 배치 및 catkin workspace 구성
   - 이 저장소를 Ubuntu 환경으로 가져옵니다.(예: `git clone https://<REPO_URL>.git`)
   - 기본 workspace인 `sim/simulator3/catkin_ws`로 이동해 의존성을 설치하고 빌드합니다.
   ```bash
   cd S13P21A507/sim/simulator3/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   source devel/setup.bash
   ```
   - 필요 시 `simulator1`, `simulator2` workspace도 동일한 방식으로 `catkin_make` 할 수 있습니다.
4. MORAI Simulator 설치 및 연동
   - MORAI에서 제공하는 설치 프로그램을 이용해 Simulator를 설치하고, 라이선스 인증을 완료합니다.
   - MORAI 데이터 디렉터리를 환경 변수에 등록합니다.
   ```bash
   echo "export MORAI_HOME=/opt/MORAI" >> ~/.bashrc
   echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$MORAI_HOME/Model" >> ~/.bashrc
   echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MORAI_HOME/lib" >> ~/.bashrc
   source ~/.bashrc
   ```
   - 프로젝트의 `sim/simulator3/catkin_ws/map_data` 및 `scripts/lib/mgeo_data`에 MORAI에서 제공한 맵 데이터를 복사합니다.
5. ROS ↔ Backend 통신 준비
   - Redis 서버가 실행 중인지 확인하고(기본: `localhost:6379`), 백엔드 `.env`에서 동일한 호스트를 가리키도록 설정합니다.
   - ROS 마스터 주소를 고정하려면 `.bashrc`에 아래 항목을 추가합니다.
   ```bash
   echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
   echo "export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
   source ~/.bashrc
   ```
   - MORAI Simulator와 차량 제어 노드를 실행할 때 `roslaunch road_block vehicle_all_monitoring_local.launch` 등을 이용해 백엔드와 WebSocket으로 연동합니다.
### Docker 환경 준비(선택)
1. 로컬 Docker 레지스트리를 사용한다면 `docker login localhost:5000`으로 인증합니다.
2. 필요 시 `.env` 파일을 `backend/.env`, `frontend/.env`로 배포 디렉터리에 복사합니다.
3. GitLab CI/CD 파이프라인을 활용할 경우 Runner에 Docker 빌드 권한과 Private Registry 접근 권한을 부여합니다.

## 프로젝트 실행

### 로컬 개발 모드
1. Redis 및 MySQL을 기동합니다.
2. Backend API 서버 실행
   ```bash
   cd backend
   . .venv/Scripts/Activate.ps1  # 또는 source .venv/bin/activate
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```
3. Celery Worker 실행(선택: 비동기 작업 사용 시)
   ```bash
   cd backend
   celery -A app.celery_app worker --loglevel=info
   ```
4. Frontend 개발 서버 실행
   ```bash
   cd frontend
   npm run dev -- --host --port 5173
   ```
   - 브라우저에서 `http://localhost:5173`으로 접속합니다.

### Docker Compose 실행
1. 루트 디렉터리에서 다음 명령을 실행합니다.
   ```bash
   docker compose up -d --build
   ```
2. Nginx가 `http://localhost`(또는 설정한 도메인)에서 프론트엔드 정적 파일을 제공하며, `/api`와 `/ws` 요청은 백엔드 컨테이너로 프록시됩니다.
3. 컨테이너 로그 확인
   ```bash
   docker compose logs -f backend
   docker compose logs -f frontend
   ```

### ROS 시뮬레이터 연동 절차
1. ROS 환경을 초기화합니다.
   ```bash
   cd sim/simulator3/catkin_ws
   source devel/setup.bash
   ```
2. ROSBridge WebSocket 서버를 띄워 백엔드와의 통신 채널을 준비합니다.
   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
3. 각 경찰차 시나리오는 개별 launch 파일로 기동합니다.
   - `sim/simulator1/catkin_ws`: 기초 주행·포위 실험용 `roslaunch road_block police_car_1.launch`
   - `sim/simulator3/catkin_ws`: 실전 시나리오용 `roslaunch road_block police_car_1.launch`, `roslaunch road_block police_car_2.launch`
   - 필요 시 추가 차량을 위해 `police_car_*.launch` 파일을 각각 별도 터미널에서 실행합니다.
4. 도주 차량(러너) 등록은 `sim/simulator2/catkin_ws`의 런처를 사용합니다.
   ```bash
   cd sim/simulator2/catkin_ws
   source devel/setup.bash
   roslaunch road_block runner_car.launch
   ```
   - 해당 런처가 ROS 토픽을 통해 백엔드 WebSocket에 등록 패킷을 전송합니다.
5. MORAI Simulator에서 대응하는 맵과 주행 시나리오를 실행해 센서 스트림을 발생시킵니다.
6. 백엔드 로그(`event_broadcasts.log`)와 프론트엔드 대시보드에서 차량 위치·상태·이벤트가 실시간 갱신되는지 확인합니다.

## 환경 변수
| 서비스 | 키 | 설명 |
|--------|----|------|
| Backend | `DB_HOST`, `DB_PORT`, `DB_USERNAME`, `DB_PASSWORD`, `DB_NAME` | MySQL 연결 정보 |
| Backend | `DB_SSL_CONFIG` | MySQL SSL 설정(JSON 문자열) |
| Backend | `HMAC_SECRET_KEY` | ROS 메시지 서명용 시크릿 |
| Backend | `REDIS_HOST` | Celery/Redis 연결 주소 |
| Frontend | `VITE_API_BASE` | REST API 기본 URL (예: `https://j13a507.p.ssafy.io/api`) |
| Frontend | `VITE_WS_BASE` | WebSocket 기본 URL (예: `wss://j13a507.p.ssafy.io/ws`) |

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

메시지 타입은 `backend/app/common/ws_codes.py`에 정의되어 있으며, `websocket_service.py`가 패킷 파싱과 브로드캐스트를 담당합니다.

## 참고 문서
- [기능 명세서](https://www.notion.so/262546b4de0280d3a988ea11e464ff57)
- [API 명세서](https://magnificent-lighter-489.notion.site/API-262546b4de0280779251d4816ce29941?pvs=74)
- [Figma Lo-Fi 와이어프레임](https://www.figma.com/design/654U42E22pFG5uiDIWTAFy/%ED%8A%B9%ED%99%94-%ED%8E%98%EC%9D%B4%EC%A7%80-%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83-%EC%B4%88%EC%95%88?node-id=0-1)







