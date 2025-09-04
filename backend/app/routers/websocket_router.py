# app/router/websocket_router.py

from fastapi import APIRouter, WebSocket
from starlette.websockets import WebSocketDisconnect

# 라우터를 생성합니다. 파일 단위로 관리하므로 prefix는 설정하지 않습니다.
router = APIRouter(
    tags=["WebSockets"],
)

# ============================================
#               Vehicle WebSockets
# ============================================

@router.websocket("/ws/vehicles")
async def websocket_vehicle_registration(websocket: WebSocket):
    """차량 등록을 위한 웹소켓"""
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            await websocket.send_text(f"Message text was: {data}")
    except WebSocketDisconnect:
        print("Client disconnected from vehicle registration")


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