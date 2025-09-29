# 경찰차 자율주행 로드블락 시스템 - Frontend

## 실행 방법

### 개발 환경

```bash
npm install
npm run dev
```

### 프로덕션 빌드

```bash
npm run build
npm run preview
```

### Docker 실행

```bash
docker compose up --build
```

## 환경 변수
 - `VITE_API_BASE` : 백엔드 API 주소
 - `VITE_WS_BASE` : WebSocket 서버 주소

```
VITE_API_BASE=http://api.example.com
VITE_WS_BASE=ws://ws.example.com
```

## 기술 스택

- **언어 및 프레임워크**
  - TypeScript
  - React
  - Vite (빌드/번들링)

- **상태 관리**
  - Zustand

- **스타일링 및 UI**
  - Tailwind CSS
  - Lucide React (아이콘)

- **지도 렌더링**
  - OpenLayers

- **통신**
  - Axios (REST API)
  - WebSocket (실시간 통신)

- **품질 관리**
  - ESLint (코드 품질 검사)

- **운영/배포**
  - Docker (컨테이너 실행)

## 폴더 구조

```
frontend/
├── public/                # 정적 파일 디렉토리
│   ├── vite.svg          
│   └── image/            # 지도 이미지 리소스
│
├── src/
│   ├── components/       # 공통 컴포넌트
│   │   ├── DashboardLayout.tsx    # 대시보드 레이아웃 
│   │   ├── InfoEventModal.tsx     # 이벤트 알림 모달
│   │   └── TestModal.tsx          # 모달 테스트용
│   │
│   ├── features/         # 주요 기능별 컴포넌트
│   │   ├── dashboard/    # 대시보드 관련 컴포넌트
│   │   │   └── components/
│   │   │       ├── ControlSidebar.tsx    # 차량 목록 및 필터링 사이드바
│   │   │       ├── VehicleListItem.tsx   # 개별 차량 정보 표시 컴포넌트
│   │   │       └── LogListItem.tsx       # 로그 항목 표시 컴포넌트
│   │   │
│   │   ├── events/       # 이벤트 로그 관련 컴포넌트
│   │   │   └── components/
│   │   │       ├── EventTable.tsx        # 이벤트 로그 테이블 컴포넌트
│   │   │       └── EventRow.tsx          # 개별 이벤트 로그 행 컴포넌트
│   │   │
│   │   ├── main/        # 메인 페이지 관련 컴포넌트
│   │   │   ├── MainLanding.tsx          # 메인 랜딩 페이지 컴포넌트
│   │   │   ├── MapCard.tsx              # 지도 선택 카드 컴포넌트
│   │   │
│   │   └── map/         # 지도 관련 컴포넌트
│   │       └── Components/
│   │           ├── MapView.tsx           # OpenLayers 기반 지도 뷰 컴포넌트
│   │           └── VehicleMarker.tsx     # 차량 위치 마커 컴포넌트
│   │
│   ├── utils/           # 유틸리티 함수 (시간 관련 UI 표시 정의)
│   ├── stores/          # Zustand 상태관리
│   ├── pages/           # 라우팅 페이지 컴포넌트
│   ├── styles/          # 공통 컴포넌트 CSS 스타일
│   ├── types.ts         # TypeScript 타입 정의
│   ├── websockets/      # WebSocket 통신 관련
│   └── Apis.ts          # axios 활용 HTTP API 통신
│
├── vite.config.ts       # Vite 설정
└── tsconfig.json        # TypeScript 설정
```

## 브라우저 지원

본 프로젝트는 최신 ECMAScript(ES2023) 기능과 WebSocket API를 활용합니다.  
따라서 다음 환경에서의 실행을 보장합니다:

- Chrome (최신 버전)
- Firefox (최신 버전)
- Safari (최신 버전)
- Edge (최신 버전)

⚠️ Internet Explorer 및 구형 브라우저는 지원하지 않습니다.  
구형 브라우저 호환이 필요할 경우, polyfill 설정 또는 `.browserslistrc` 구성을 추가해야 합니다.