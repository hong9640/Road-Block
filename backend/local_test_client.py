import asyncio
import websockets
import struct
import hmac
import hashlib
import os
from dotenv import load_dotenv

# --- 테스트 설정 ---
# 로컬에서 실행 중인 FastAPI 서버의 웹소켓 주소입니다.
WEBSOCKET_URI = "ws://localhost:8000/ws/vehicles"

# 서버로 보낼 차량 정보입니다. 값을 바꿔가며 테스트할 수 있습니다.
VEHICLE_TYPE = 1  # 0: POLICE, 1: RUNNER
CAR_NAME = "TEST_CAR_01"  # 10바이트를 넘지 않도록 주의

# --- 테스트 클라이언트 메인 함수 ---
async def send_registration_request():
    """서버에 차량 등록 요청을 보내고 응답을 확인하는 테스트 클라이언트입니다."""
    
    print("--- WebSocket Test Client Started ---")
    
    # 1. .env 파일에서 HMAC 비밀키를 로드합니다.
    load_dotenv()
    secret_key_str = os.getenv("HMAC_SECRET_KEY")
    if not secret_key_str:
        print("\n[ERROR] .env 파일에 HMAC_SECRET_KEY가 설정되지 않았습니다.")
        print("프로젝트 루트에 .env 파일을 생성하고 키를 추가해주세요.")
        return
    secret_key = secret_key_str.encode('utf-8')
    print("[INFO] HMAC secret key loaded successfully.")

    # 2. 전송할 바이너리 패킷의 헤더(앞 12바이트)를 생성합니다.
    #    - car_name을 UTF-8로 인코딩하고, 10바이트가 되도록 NULL(b'\x00')로 채웁니다(padding).
    car_name_bytes = CAR_NAME.encode('utf-8').ljust(10, b'\x00')
    
    #    - big-endian(>), uchar(B), uchar(B), 10-char string(10s) 형식으로 데이터를 압축(pack)합니다.
    header = struct.pack('>BB10s', 0xA0, VEHICLE_TYPE, car_name_bytes)

    # 3. 생성된 헤더를 기반으로 HMAC 인증 코드를 계산합니다.
    hmac_val = hmac.new(secret_key, header, hashlib.sha256).digest()[:16]
    
    # 4. 최종 전송할 패킷을 만듭니다 (헤더 + HMAC). 총 28바이트.
    packet_to_send = header + hmac_val
    
    print(f"\n[INFO] Sending Packet ({len(packet_to_send)} bytes):")
    print(f"       - Hex: {packet_to_send.hex()}")
    print("-" * 35)

    # 5. 웹소켓 서버에 연결하고 패킷을 전송합니다.
    try:
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            await websocket.send(packet_to_send)
            print("[SUCCESS] Packet sent to server.")
            
            # 6. 서버로부터 응답을 기다립니다.
            print("[INFO] Waiting for server response...")
            response = await websocket.recv()
            
            # 7. 받은 응답을 분석하고 출력합니다.
            print(f"\n[SUCCESS] Received Response ({len(response)} bytes):")
            print(f"          - Raw: {response}")
            print(f"          - Hex: {response.hex()}")

            # 응답 데이터 형식에 따라 파싱하여 내용을 확인합니다.
            if len(response) == 5 and response[0] == 0xA1: # REGISTER_SUCCESS
                msg_type, vehicle_id = struct.unpack('>BI', response)
                print(f"\n[INFO] Response Parsed: REGISTER_SUCCESS")
                print(f"       - New Vehicle ID: {vehicle_id}")
            elif len(response) == 18 and response[0] == 0x02: # NACK_ERROR
                msg_type, err_code, _ = struct.unpack('>BB16s', response)
                print(f"\n[INFO] Response Parsed: NACK_ERROR")
                print(f"       - Error Code: {err_code}")
            else:
                 print("\n[WARNING] Received an unknown format response.")

    except websockets.exceptions.ConnectionClosedError as e:
        print(f"\n[ERROR] Connection failed: {e}. Is the server running?")
    except Exception as e:
        print(f"\n[ERROR] An unexpected error occurred: {e}")
    
    print("\n--- Test Finished ---")


if __name__ == "__main__":
    # 필요한 라이브러리가 설치되어 있는지 확인합니다.
    try:
        import websockets
        import dotenv
    except ImportError:
        print("필요한 라이브러리가 설치되지 않았습니다. pip install websockets python-dotenv 명령어를 실행해주세요.")
    else:
        asyncio.run(send_registration_request())

    
