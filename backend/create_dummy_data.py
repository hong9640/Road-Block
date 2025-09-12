import struct
import hmac
import hashlib
import os
import sys
import asyncio
from dotenv import load_dotenv

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

# 2. dotenv 라이브러리를 임포트합니다.
from dotenv import load_dotenv

# 3. .env 파일의 절대 경로를 직접 계산하여 load_dotenv에 전달합니다.
#    이렇게 하면 스크립트를 어디서 실행하든 경로 문제가 발생하지 않습니다.
dotenv_path = os.path.join(project_root, '.env')
if os.path.exists(dotenv_path):
    load_dotenv(dotenv_path)
    print(f"✅ .env 파일 로드 성공: {dotenv_path}")
else:
    # 이 메시지가 보인다면 .env 파일 이름이나 위치에 정말로 문제가 있는 것입니다.
    raise FileNotFoundError(f"❌ .env 파일을 찾을 수 없습니다: {dotenv_path}")

# --- DB 및 프로젝트 모듈 임포트 ---
from app.db import AsyncSessionMaker, get_vehicle_by_ros_id, save_vehicle_location

# .env 파일에서 환경 변수를 로드합니다.
load_dotenv()

# --- 설정 ---
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError("HMAC_SECRET_KEY가 .env 파일에 설정되지 않았습니다.")
SECRET_KEY = SECRET_key_str.encode('utf-8')


def _calculate_hmac(data: bytes) -> bytes:
    """서버와 동일한 방식으로 HMAC-SHA256 값을 계산합니다 (16바이트로 자름)."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

def create_location_packet(vehicle_id: int, x: float, y: float) -> bytes:
    """차량 위치 정보를 담은 바이너리 패킷을 생성합니다."""
    header = struct.pack('<Iff', vehicle_id, x, y)
    hmac_signature = _calculate_hmac(header)
    packet = header + hmac_signature
    return packet

async def main():
    """메인 비동기 실행 함수"""
    # --- 더미 데이터 정의 ---
    # id가 6인 차량의 vehicle_id(ROS ID)가 6이라고 가정합니다.
    # 만약 다르다면 이 값을 실제 vehicle_id로 변경해야 합니다.
    TARGET_VEHICLE_ID = 6
    DUMMY_POS_X = 37.1234
    DUMMY_POS_Y = 127.5678
    
    # 1. 더미 데이터로 바이너리 패킷 생성
    dummy_packet = create_location_packet(TARGET_VEHICLE_ID, DUMMY_POS_X, DUMMY_POS_Y)
    
    print("--- 위치 정보 더미 데이터 생성 결과 ---")
    print(f"대상 차량 ID (ROS): {TARGET_VEHICLE_ID}")
    print(f"좌표: ({DUMMY_POS_X}, {DUMMY_POS_Y})")
    print(f"생성된 패킷 길이: {len(dummy_packet)} bytes")
    print(f"생성된 패킷 (Hex): {dummy_packet.hex()}")

    # 2. 생성된 위치 정보를 DB에 저장
    print("\n--- 데이터베이스에 위치 정보 저장 시작 ---")
    try:
        async with AsyncSessionMaker() as session:
            # ROS ID를 사용해 DB에서 차량의 내부 PK(id)를 찾습니다.
            target_vehicle = await get_vehicle_by_ros_id(session, TARGET_VEHICLE_ID)
            
            if target_vehicle:
                # `save_vehicle_location` 함수는 차량의 내부 PK를 필요로 합니다.
                success = await save_vehicle_location(
                    session,
                    vehicle_pk_id=target_vehicle.id,
                    pos_x=DUMMY_POS_X,
                    pos_y=DUMMY_POS_Y
                )
                if success:
                    print(f"✅ 성공: Vehicle(id={target_vehicle.id})의 위치 정보를 DB에 저장했습니다.")
                else:
                    print("❌ 실패: 위치 정보 저장에 실패했습니다.")
            else:
                print(f"❌ 실패: DB에서 vehicle_id가 {TARGET_VEHICLE_ID}인 차량을 찾을 수 없습니다.")

    except Exception as e:
        print(f"❌ DB 작업 중 에러 발생: {e}")


# --- 스크립트 실행 ---
if __name__ == "__main__":
    # 비동기 main 함수를 실행합니다.
    asyncio.run(main())