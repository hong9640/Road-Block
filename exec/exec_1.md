# Gitlab 소스 클론 이후 빌드 및 배포할 수 있도록 정리한 문서

## 사용한 jvm, 웹서버, was 제품 등의 종류와 설정 값, 버전(IDE버전 포함)
- Python 인터프리터 : Python 3.11.13
- 웹서버 (Web Server): Nginx 1.29.1
- WAS : Uvicorn 0.35.0
- 컨테이너 : Docker Engine 27.5.2
- IDE : Visual Studio Code 1.104.1

## 빌드 시 사용되는 환경 변수 등의 내용
### 서비스 환경 세팅
1. **프로젝트 세팅**  
    ```
    .
    ├── .gitlab-ci.yml
    ├── docker-compose.yml
    ├── backend
    │   ├── .env
    │   ├── Dockerfile
    │   ├── requirements.txt
    │   └── app
    │       ├── main.py
    │       ├── db.py
    │       ├── db_sync.py
    │       ├── celery_app.py
    │       └── tasks.py
    ├── frontend
    │   ├── Dockerfile
    │   ├── nginx.conf
    │   └── .env
    └── nginx
        └── default.conf
    ```


2. **환경 변수 설정**
    - `./frontend/.env`
        ```
        # 프로젝트를 세팅한 폴더 경로
        VITE_API_BASE="https://j13a507.p.ssafy.io/api"
        VITE_WS_BASE="wss://j13a507.p.ssafy.io/ws"

        ```
    - `./backend/app/.env`
        ```
        # DB 접속 정보
        DB_HOST=localhost
        DB_PORT=3306
        DB_USERNAME=user
        DB_PASSWORD=password
        DB_NAME=dbname
        DB_SSL_CONFIG = '{"ssl_true": true}'

        # 웹소켓 hmac 생성용 시크릿 키
        HMAC_SECRET_KEY = ""

        # Redis 접속 정보(단, 배포가 아닌 개발 환경에서는 삭제!)
        REDIS_HOST=redis
        ```
## 빌드 및 실행 방법
1. **초기 로컬 개발 환경 가이드**
    - `git pull origin <브랜치명>`
    - `.env` 작성
    - `cd backend`
    - `python -m venv venv` (가상환경 생성)
    - `venv\Scripts\activate` (가상환경 활성화)
    - `pip install -r requirements.txt` (라이브러리 설치)
    - `uvicorn app.main:app --reload` (서버 실행)
    <hr>
2. **Redis 설치 및 자동 실행 설정 (Ubuntu/WSL)**
    - `sudo apt-get update`
    - `sudo apt-get install redis-server`  
    **WSL을 재시작해도 Redis가 자동으로 켜지도록 설정**
    - `sudo systemctl enable redis-server`  
    **Redis 최초 실행**
    - `sudo systemctl start redis-server`
    <hr>
3. **Celery 실행**
    - fastapi 터미널 옆에 추가 터미널 생성
    - `cd backend`
    - `venv\Scripts\activate` (가상환경 활성화)
    - `celery -A app.celery_app worker -P solo --loglevel=info`(Celery 워커 실행)
    <hr>
4. **서버 종료**
    - `ctrl + C` (서버 종료)
    - `git add .` -> `git commit -m "message"` -> `git push origin <브렌치명>`
    <hr>
5. **EC2 가이드**
    - `MobaXterm` 접속 및 세션 연결
        - `MobaXterm` 왼쪽 위의 `Session` 버튼을 클릭합니다.
        - 새로 나온 창에서 SSH 아이콘을 클릭합니다.
        - `SSH` 설정 창에서 아래 정보들을 입력합니다.
        - `Remote host`: EC2 인스턴스의 Public IP 주소 또는 도메인 주소(예: j13a507.p.ssafy.io)를 입력합니다.
        - `Specify username`: 체크박스를 선택하고 사용자 이름(예: ubuntu)을 입력합니다.
        - `Advanced SSH settings` 탭을 클릭한 뒤, `Use private key` 옵션을 체크하고, 옆의 파일 아이콘을 눌러 EC2 접속용 `.pem` 키 파일을 선택합니다.
        - OK 버튼을 누르면 세션이 생성되고 자동으로 서버에 접속됩니다.
    - `User Sessions`에 생성된 세션에 접속, git pull 한 파일 좌측 home/ubuntu/ 경로에 붙여넣기
    - `python3.11 -m venv <가상환경 이름>` (가상환경 생성) **주의!!** 반드시 `python3.11` 이걸로 생성해야 합니다! 아니면 `pip install` 안됩니다!!
    - `source <가상환경 이름>/bin/activate` (새로운 가상환경 실행)
    - `pip install -r requirements.txt` (라이브러리 설치)
    - `sudo /home/ubuntu/S13P21A507/backend/<가상환경 이름>/bin/uvicorn app.main:app --host 0.0.0.0 --port <개인 개발 포트>`(서버 실행)
6. **EC2 celery 설정**
    - `backend/app/.env`에서 `REDIS_HOST=redis` 삭제  
    - 동일한 세션의 터미널 추가  
    - `cd backend`  
    - `source <가상환경 이름>/bin/activate` (새로운 가상환경 실행)  
    - `celery -A app.celery_app worker --loglevel=info`(Celery 워커 실행)
7. **EC2 서버 종료**
    - `ctrl + C` (서버 종료, 2개의 터미널 모두 종료)  
    
6. **서비스 배포**
    - `CI (Continuous Integration)`: 이미지 자동 빌드 및 푸시
        - 개발자가 로컬 PC에서 git push를 실행하면, GitLab의 CI 파이프라인이 자동으로 실행됩니다. 이 파이프라인은 최신 코드를 기반으로 새로운 Docker 이미지를 빌드하고, EC2 서버 내 `Private Registry`에 저장(push)합니다.

    - `CD (Continuous Deployment)`: 서비스 수동 배포
        - 서버 관리자가 EC2 서버에 접속하여, 아래의 `docker-compose` 명령어들을 순서대로 실행하여 수동으로 서비스를 업데이트합니다.

            - `docker-compose pull`  
            Private Registry에 저장된 최신 이미지를 서버로 가져옵니다.

            - `docker-compose down`  
            현재 실행 중인 구버전의 컨테이너를 안전하게 중지하고 삭제합니다.

            - `docker-compose up -d` 
            방금 내려받은 최신 이미지로 새로운 컨테이너를 백그라운드에서 실행합니다.
    

## DB 접속 정보 등 프로젝트(ERD)에 활용되는 주요 계정 및 프로퍼티가 정의된 파일 목록
### .env
- 데이터베이스 접속 정보(사용자, 비밀번호, 호스트, 포트) 등 모든 민감 정보를 정의하는 파일입니다. 이 파일은 .gitignore에 추가하여 Git 저장소에 포함되지 않도록 관리합니다.

### app/db.py
- .env 파일의 변수를 읽어 FastAPI의 비동기 이벤트 루프와 함께 동작하는 비동기 데이터베이스 엔진과 세션을 생성하고, 앱 전체에 DB 연결을 제공합니다.

### app/db_sync.py
- .env 변수를 읽어 Celery 워커와 같은 동기 환경을 위한 DB 엔진과 세션을 생성합니다. 비동기 환경인 FastAPI와 격리된 DB 연결을 제공하기 위해 별도로 존재합니다.

### app/celery_app.py
- Celery 애플리케이션의 설정을 담당하는 중심 파일입니다. 메시지 브로커(Broker)와 결과 백엔드(Backend)로 Redis를 사용하도록 지정하며, app.tasks와 같은 파일에 정의된 함수들이 Task로 등록될 수 있도록 Celery 인스턴스를 생성하고 제공합니다.

### app/tasks.py
- 실제 백그라운드에서 처리될 작업의 로직이 정의된 파일입니다. Celery 워커는 이 파일에 정의된 함수들을 실행하여 DB 저장과 같은 시간이 걸리는 작업을 처리합니다.

### Dockerfile
- 컨테이너 이미지를 생성하기 위한 설계도 또는 레시피 파일입니다. 각 서비스(프론트엔드, 백엔드)가 독립된 환경에서 일관되게 동작할 수 있도록 필요한 모든 설정과 명령을 정의합니다.

### docker-compose.yml
- backend, frontend, nginx 등 프로젝트를 구성하는 모든 서비스를 정의하고 연결합니다. 특히, env_file 속성을 통해 .env 파일의 내용을 각 서비스 컨테이너의 환경 변수로 주입하는 역할을 합니다.

### nginx/default.conf
- 프로젝트의 메인 리버스 프록시 역할을 수행합니다. 외부의 모든 HTTP/HTTPS 요청(port 80, 443)을 최초로 받아, 경로에 맞춰 적절한 내부 서비스 컨테이너(백엔드, 프론트엔드)로 전달하는 게이트웨이입니다.

### frontend/nginx.conf
- 프론트엔드 Docker 컨테이너 내부에서 실행되는 Nginx의 설정 파일입니다. 빌드된 정적 파일(HTML, CSS, JS)을 제공하고, SPA(Single Page Application)의 라우팅이 새로고침 시에도 정상 작동하도록 모든 요청을 index.html로 리다이렉트하는 역할을 합니다.