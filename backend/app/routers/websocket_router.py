from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.services import websocket_service
import traceback
# --- 연결 관리자 ---
class ConnectionManager:
    """웹소켓 연결을 관리하는 범용 매니저"""
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: bytes):
        """연결된 모든 클라이언트에게 메시지를 브로드캐스트합니다."""
        disconnected_sockets = []
        for connection in self.active_connections:
            try:
                await connection.send_bytes(message)
            except WebSocketDisconnect:
                disconnected_sockets.append(connection)
        
        # 브로드캐스트 중 연결이 끊어진 소켓들을 정리합니다.
        for socket in disconnected_sockets:
            self.disconnect(socket)

# 모든 클라이언트에게 이벤트를 전파하기 위한 단일 브로드캐스트 매니저
broadcast_manager = ConnectionManager()
router = APIRouter(tags=["WebSockets"])

# --- 웹소켓 엔드포인트 ---

@router.websocket("/ws/vehicles")
async def websocket_vehicle_register(websocket: WebSocket):
    """
    차량 등록 요청을 처리하는 웹소켓.
    ROS의 요청에 직접 응답하고, 모든 클라이언트에게 등록 이벤트를 브로드캐스트합니다.
    """
    await broadcast_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            ros_response, front_event = await websocket_service.handle_vehicle_registration(data)
            
            # 1. 요청을 보낸 ROS 클라이언트에게 직접 응답합니다.
            await websocket.send_bytes(ros_response)

            # 2. 프론트엔드로 보낼 이벤트가 있다면 전체 브로드캐스트합니다.
            if front_event:
                await broadcast_manager.broadcast(front_event)

    except WebSocketDisconnect:
        print("Client disconnected from registration endpoint.")
        broadcast_manager.disconnect(websocket)
    except Exception as e:
        print("--- !!! Unhandled error in registration WebSocket !!! ---")
        print(f"Error Type: {type(e)}")
        print(f"Error Details: {e}")
        # traceback을 출력하면 어느 파일, 몇 번째 줄에서 에러가 났는지 정확히 알 수 있습니다.
        traceback.print_exc()
        print("---------------------------------------------------------")
        broadcast_manager.disconnect(websocket)


@router.websocket("/ws/vehicles/{vehicle_id}/location")
async def websocket_vehicle_location_update(websocket: WebSocket, vehicle_id: int):
    """
    특정 차량의 위치 정보 업데이트를 처리하고 모든 클라이언트에게 브로드캐스트합니다.
    """
    await broadcast_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            front_event = await websocket_service.handle_location_update(data)

            # 서비스 로직에서 생성된 위치 이벤트가 있다면 전체 브로드캐스트합니다.
            if front_event:
                await broadcast_manager.broadcast(front_event)
                
    except WebSocketDisconnect:
        print(f"Client for vehicle {vehicle_id} location updates disconnected.")
        broadcast_manager.disconnect(websocket)
    except Exception as e:
        print(f"Unhandled error in location WebSocket for vehicle {vehicle_id}: {e}")
        broadcast_manager.disconnect(websocket)


@router.websocket("/ws/vehicles/{vehicle_id}/status")
async def websocket_vehicle_status_update(websocket: WebSocket, vehicle_id: int):
    """
    차량 상태 정보 업데이트를 처리하고, 결과를 모든 클라이언트에게 브로드캐스트합니다.
    에러 발생 시 요청을 보낸 클라이언트에게 직접 NACK을 보냅니다.
    """
    await broadcast_manager.connect(websocket)
    print(f"Client connected for status updates of vehicle {vehicle_id}")
    try:
        while True:
            data = await websocket.receive_bytes()
            # 서비스 핸들러를 호출하여 (ROS 응답, 프론트엔드 이벤트)를 받습니다.
            ros_response, front_event = await websocket_service.handle_vehicle_status_update(data)

            # 1. ROS(임베디드) 클라이언트에게 보낼 에러 응답(NACK)이 있다면 보냅니다.
            # (성공 시에는 별도 응답이 없습니다.)
            if ros_response:
                await websocket.send_bytes(ros_response)

            # 2. 프론트엔드 클라이언트에게 브로드캐스트할 이벤트가 있다면 보냅니다.
            if front_event:
                await broadcast_manager.broadcast(front_event)

    except WebSocketDisconnect:
        print(f"Client disconnected from vehicle {vehicle_id} status updates")
        broadcast_manager.disconnect(websocket)
    except Exception as e:
        print(f"Unhandled error in status WebSocket for vehicle {vehicle_id}: {e}")
        broadcast_manager.disconnect(websocket)



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