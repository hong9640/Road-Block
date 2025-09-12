# app/routers/websocket_router.py
from app.common.ws_codes import MessageType
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.services import websocket_service
from app.schemas import websocket_schema # 스키마 임포트
import traceback
from typing import Dict

# --- 연결 관리자 ---
class ConnectionManager:
    """웹소켓 연결을 관리하는 범용 매니저"""
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"Client connected. Total clients: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            print(f"Client disconnected. Total clients: {len(self.active_connections)}")

    async def broadcast(self, message: bytes):
        """연결된 모든 클라이언트에게 메시지를 브로드캐스트합니다."""
        disconnected_sockets = []
        for connection in self.active_connections:
            try:
                await connection.send_bytes(message)
            except WebSocketDisconnect:
                disconnected_sockets.append(connection)
        
        for socket in disconnected_sockets:
            self.disconnect(socket)

# --- (수정) 차량 및 맵 이벤트용 매니저 분리 및 관리 ---
# 차량 관련 모든 이벤트를 위한 글로벌 매니저
vehicle_manager = ConnectionManager()
# 맵 ID별로 클라이언트 그룹을 관리하기 위한 딕셔너리
map_managers: Dict[str, ConnectionManager] = {}

router = APIRouter(tags=["WebSockets"])

# --- (수정) 차량 관련 엔드포인트에서 vehicle_manager 사용 ---

@router.websocket("/ws/vehicles")
async def unified_vehicle_websocket(websocket: WebSocket):
    """
    차량 관련 모든 웹소켓 요청(등록, 위치, 상태)을 처리하는 통합 엔드포인트.
    수신된 바이너리 데이터의 첫 번째 바이트(MessageType)를 기준으로 작업을 분기합니다.
    """
    await vehicle_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if not data:
                # 비어있는 데이터는 무시
                continue

            # --- 핵심 로직: 첫 바이트로 메시지 타입 식별 ---
            message_type = data[0] 
            
            # 1. 차량 등록 요청 처리
            if message_type == MessageType.EVENT_VEHICLE_REGISTERED:
                ros_response, front_event = await websocket_service.handle_vehicle_registration(data)
                # ROS 응답은 요청을 보낸 클라이언트에게만 전송
                if ros_response:
                    await websocket.send_bytes(ros_response)
                # 프론트엔드 이벤트는 모두에게 브로드캐스트
                if front_event:
                    await vehicle_manager.broadcast(front_event)

            # 2. 차량 위치 업데이트 처리
            # NOTE: 클라이언트가 위치 정보 전송 시 MessageType을 포함해야 합니다. (아래 설명 참고)
            elif message_type == MessageType.POSITION_BROADCAST_2D: # 위치 업데이트용 MessageType, 필요시 ws_codes에 정의
                front_event = await websocket_service.handle_location_update(data)
                if front_event:
                    await vehicle_manager.broadcast(front_event)
            
            # 3. 차량 상태 업데이트 처리
            elif message_type == MessageType.STATE_UPDATE:
                ros_response, front_event = await websocket_service.handle_vehicle_status_update(data)
                if ros_response:
                    await websocket.send_bytes(ros_response)
                if front_event:
                    await vehicle_manager.broadcast(front_event)
            
            else:
                print(f"Unknown message type received on /ws/vehicles: {message_type}")

    except WebSocketDisconnect:
        print("Client disconnected from unified vehicle endpoint.")
        vehicle_manager.disconnect(websocket)
    except Exception as e:
        print(f"An error occurred in the unified vehicle endpoint: {e}")
        traceback.print_exc()
        vehicle_manager.disconnect(websocket)

# ============================================
#               Map Event WebSockets
# ============================================

@router.websocket("/ws/maps/{map_id}/events")
async def websocket_map_events(websocket: WebSocket, map_id: str):
    """
    맵 관련 모든 이벤트를 처리하고 해당 맵 채널의 클라이언트에게 브로드캐스트합니다.
    (예: 디버깅용 프론트엔드에서 JSON 이벤트를 보내면 바이너리로 변환하여 브로드캐스트)
    """
    # 맵 ID에 해당하는 매니저가 없으면 새로 생성합니다.
    if map_id not in map_managers:
        map_managers[map_id] = ConnectionManager()
    manager = map_managers[map_id]
    await manager.connect(websocket)

    try:
        while True:
            # 프론트엔드(또는 테스트 클라이언트)로부터 JSON 형식의 제어 메시지를 받습니다.
            event_data = await websocket.receive_json()
            event_type = event_data.get("type")
            payload = event_data.get("payload", {})
            binary_packet = None

            if event_type == "start_tracking":
                try:
                    # JSON 데이터를 Pydantic 모델로 변환
                    event_model = websocket_schema.StartTrackingEvent(**payload)
                    # 서비스 함수를 호출하여 바이너리 패킷 생성
                    binary_packet = websocket_service.create_start_tracking_packet(event_model)
                    print(f"[{map_id}] Broadcasting 'Start Tracking' event.")
                except Exception as e:
                    print(f"Error processing 'start_tracking': {e}")

            elif event_type == "capture_success":
                try:
                    event_model = websocket_schema.CaptureSuccessEvent(**payload)
                    binary_packet = websocket_service.create_capture_success_packet(event_model)
                    print(f"[{map_id}] Broadcasting 'Capture Success' event.")
                except Exception as e:
                    print(f"Error processing 'capture_success': {e}")

            elif event_type == "tracking_failed":
                try:
                    event_model = websocket_schema.CatchFailedEvent(**payload)
                    binary_packet = websocket_service.create_catch_failed_packet(event_model)
                    print(f"[{map_id}] Broadcasting 'Tracking Failed' event.")
                except Exception as e:
                    print(f"Error processing 'tracking_failed': {e}")

            else:
                print(f"[{map_id}] Received unknown event type: {event_type}")

            # 생성된 바이너리 패킷이 있다면 현재 맵의 모든 클라이언트에게 브로드캐스트
            if binary_packet:
                await manager.broadcast(binary_packet)

    except WebSocketDisconnect:
        print(f"Client disconnected from map '{map_id}' events.")
        manager.disconnect(websocket)
    except Exception as e:
        print(f"--- Unhandled error in map WebSocket for '{map_id}' ---")
        traceback.print_exc()
        print("---------------------------------------------------------")
        manager.disconnect(websocket)