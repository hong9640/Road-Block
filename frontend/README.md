## 1. 사용한 기술 스택

- React
- Vite
- GeoJson
- Zustand
- IndexedDB


## 2. 폴더 구조
```
src/
 ├─ assets/                  # 이미지, 아이콘, 스타일 등 정적 자원
 │
 ├─ components/              # Atomic Design 기반 공통 UI 컴포넌트
 │   ├─ atoms/
 │   │   ├─ SmallBtn.tsx
 │   │   ├─ CircleBtn.tsx
 │   │   └─ Marker.tsx       # 차량 Marker 공통 스타일
 │   │
 │   ├─ molecules/           # 입력창, 카드, 리스트 아이템 등
 │   │   ├─ LogListItem.tsx         # Side Panel 사건 기록 Item
 │   │   └─ VehicleListItem.tsx     # Side Panel 경찰 차량 정보 Item
 │   │
 │   ├─ organisms/           # 지도 UI, 로그 테이블, 대시보드 레이아웃 등 (Layout 위주)
 │   │   ├─ MapView.tsx             # 공통 Map UI Section
 │   │   ├─ EventTable.tsx          # 사건 기록 모음 Page 오른쪽 Section,
 │   │   └─ DashboardLayout.tsx     # 대시보드 Page 왼쪽 Section
 │   │
 │   └─ templates/           # 페이지 공통 레이아웃
 │       └─ MainTemplate.tsx        # 대시보드 전환 관련 공통 Layout
 │
 ├─ features/                # 도메인/기능 단위 폴더
 │   ├─ main/                # 메인 페이지
 │   │   ├─ MainLanding.tsx
 │   │   ├─ MapCards.ts             
 │   │   └─ navLinks.ts             
 │   │ 
 │   ├─ map/                 # 지도 관련 기능
 │   │   ├─ VehicleMarker.tsx       # 차량 표시 특화 Marker
 │   │   ├─ useMapData.ts           # 차량 위치 데이터 (WS) 반영
 │   │   └─ smoothing.ts            # 차량 위치 UI Jumping 방지용 Logic
 │   │
 │   ├─ dashboard/           # 관제/일반 전환 대시보드
 │   │   ├─ ControlSidebar.tsx 
 │   │   ├─ useDashboardState.ts    # 대시보드 필터
 │   │   └─ modeSwitch.ts           
 │   │
 │   └─ events/              # 사건 로그 조회/필터
 │       ├─ EventRow.tsx
 │       ├─ useEventData.ts
 │       └─ formatEvent.ts
 │
 ├─ pages/                   # 최종 라우팅 단위 페이지
 │   ├─ MapPage.tsx          # 지도 페이지
 │   ├─ DashboardPage.tsx    # 관제 대시보드 페이지 (전환 기능 포함)
 │   ├─ MainPage.tsx         # 사이트 진입 시 메인 화면
 │   └─ EventsPage.tsx       # 사건 로그 페이지
 │
 │
 └─ utils/                   # 전역 유틸 함수
     ├─ api.ts
     └─ WSconnection.ts      # WS 연결 관리 모듈
```

1. `features/[name]/hooks/` **(커스텀 훅 전용)**

    특정 **기능(Feature)**에 밀접하게 관련된 상태 관리 + 비즈니스 로직을 캡슐화합니다. <br>
    React의 `useState` , `useEffect` , `useMemo` 등을 조합해서 UI와 분리된 재사용 가능한 로직을 만듭니다. <br>
    네이밍은 반드시 `useSomething.ts` 로 시작합니다.

    - `useMapData.ts`
    → WebSocket을 통해 차량 위치 데이터를 받아오고, 가공 후 반환.
    - `useDashboardState.ts`
    → 대시보드의 필터(차량 선택, 시간 범위 등) 상태 관리.
    - `useEventData.ts`
    → API에서 사건 로그를 불러와 상태로 관리, Pagenation 로직 포함.

2. `features/[name]/utils/` (순수 유틸 함수)

    특정 Feature에서만 사용하는 순수 함수(Pure Function) 모음입니다. <br>
    React에 의존하지 않고, 입력값 → 출력값을 만드는 가벼운 로직이 여기에 작성됩니다. 훅 안/밖 어디서든 호출 가능합니다.

    - `smoothing.ts`
    → 차량 위치 데이터의 오차를 보정하는 스무딩 알고리즘.
    - `modeSwitch.ts`
    → 일반 모드 ↔ 관제 모드 전환 시 필요한 데이터 변환 함수.
    - `formatEvent.ts`
    → 사건 로그의 날짜, 이벤트 타입을 보기 좋은 문자열로 변환.