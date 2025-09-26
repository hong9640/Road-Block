# 경찰차 자율주행 로드블락 시스템 - Frontend

자율주행 경찰차를 통한 도주 차량 실시간 추적/검거 모니터링 시스템입니다.

## 주요 기능

1. 실시간 차량 위치 추적
   - WebSocket을 통한 실시간 차량 위치 업데이트
   - OpenLayers 지도 상 마커 표시

2. 차량 상태 모니터링
   - 연료량, 파손상태 등 실시간 상태 확인
   - 차량별 필터링 및 검색

3. 이벤트 로깅
   - 도주/검거 등 주요 이벤트 실시간 알림
   - 이벤트 히스토리 조회

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

## 기술 스택

- 언어: TypeScript
- 프레임워크: React
- 빌드도구: Vite
- 스타일링: Tailwind CSS
- 코드품질: ESLint

## 사용 라이브러리

- **Core** : 
  - React 19
  - TypeScript
  - Vite (프로젝트 관리)

- **상태관리**
  - Zustand

- **스타일링**
  - Tailwind CSS (빠른 스타일링 라이브러리)
  - Lucide React (아이콘)

- **지도**
  - OpenLayers (가상 시뮬레이션 맵 렌더링)

- **통신**
  - Axios (비동기 처리)
  - WebSocket (Native, 실시간 통신)

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
│   │   │   └── MapCard.css              # 지도 카드 스타일링
│   │   │
│   │   └── map/         # 지도 관련 컴포넌트
│   │       └── Components/
│   │           ├── MapView.tsx           # OpenLayers 기반 지도 뷰 컴포넌트
│   │           └── VehicleMarker.tsx     # 차량 위치 마커 컴포넌트
│   │
│   ├── lib/             # 유틸리티 함수 (시간 관련 UI 표시 정의)
│   ├── stores/          # Zustand 상태관리
│   ├── pages/           # 라우팅 페이지 컴포넌트
│   ├── styles/          # 공통 컴포넌트 CSS 스타일
│   ├── types.ts         # TypeScript 타입 정의
│   ├── websockets/      # WebSocket 통신 관련
│   └── Apis.ts          # REST API 통신
│
├── vite.config.ts       # Vite 설정
└── tsconfig.json        # TypeScript 설정
```

## 환경 변수

```
VITE_API_BASE=백엔드 API 주소
VITE_WS_BASE=WebSocket 서버 주소
```

## 브라우저 지원

- 최신 버전의 Chrome, Firefox, Safari, Edge 지원
- WebSocket, ES2023 기능 지원 필요
