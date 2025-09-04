from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.services import websocket_service

# --- 연결 관리자 ---
class ConnectionManager:
    def __init__(self):
        # 지금은 모든 클라이언트를 하나의 리스트에서 관리합니다.
        # 추후 프론트엔드/임베디드 구분이 필요하면 self.front_connections 등으로 나눌 수 있습니다.
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: bytes):
        for connection in self.active_connections:
            await connection.send_bytes(message)

manager = ConnectionManager()
router = APIRouter(tags=["WebSockets"])

# --- 웹소켓 엔드포인트 ---

@router.websocket("/ws/vehicles")
async def websocket_vehicle_register(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            
            ros_response, front_event = await websocket_service.handle_vehicle_registration(data)

            # 1. 요청한 ROS 클라이언트에게 직접 응답
            await websocket.send_bytes(ros_response)

            # 2. 프론트엔드용 이벤트/에러가 있다면 전체 브로드캐스트
            if front_event:
                await manager.broadcast(front_event)

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"Unhandled error in WebSocket: {e}")
        manager.disconnect(websocket)

@router.websocket("/ws/vehicles/{vehicle_id}/location")
async def websocket_vehicle_location_update(websocket: WebSocket, vehicle_id: str):
    """차량 위치 정보 업데이트를 위한 웹소켓"""
    await websocket.accept()
    print(f"Client connected for location updates of vehicle {vehicle_id}")
    try:
        while True:
            location_data = await websocket.receive_json()
            print(f"Received location for {vehicle_id}: {location_data}")
    except WebSocketDisconnect:
        print(f"Client disconnected from vehicle {vehicle_id} location updates")


@router.websocket("/ws/vehicles/{vehicle_id}/status")
async def websocket_vehicle_status_update(websocket: WebSocket, vehicle_id: str):
    """차량 상태 정보 업데이트를 위한 웹소켓"""
    await websocket.accept()
    print(f"Client connected for status updates of vehicle {vehicle_id}")
    try:
        while True:
            status_data = await websocket.receive_json()
            print(f"Received status for {vehicle_id}: {status_data}")
    except WebSocketDisconnect:
        print(f"Client disconnected from vehicle {vehicle_id} status updates")


# ============================================
#               Map Event WebSockets
# ============================================

@router.websocket("/ws/maps/{map_id}/events")
async def websocket_map_events(websocket: WebSocket, map_id: str):
    """맵 관련 모든 이벤트를 처리하는 웹소켓"""
    await websocket.accept()
    print(f"Client connected for map events on map {map_id}")
    try:
        while True:
            event_data = await websocket.receive_json()
            event_type = event_data.get("type")

            if event_type == "start_tracking":
                print(f"[{map_id}] Tracking started: {event_data}")
            elif event_type == "tracking_failed":
                print(f"[{map_id}] Tracking failed: {event_data}")
            elif event_type == "capture_success":
                print(f"[{map_id}] Capture succeeded: {event_data}")
            else:
                print(f"[{map_id}] Unknown event type: {event_data}")

    except WebSocketDisconnect:
        print(f"Client disconnected from map {map_id} events")