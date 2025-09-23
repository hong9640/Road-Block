# create_dummy_data.py

import struct
import hmac
import hashlib
import os
from dotenv import load_dotenv

# --- .env 파일 로드 ---
load_dotenv()

# --- ⚙️ 설정: 이 부분을 수정하여 테스트하세요 ---
# .env 파일에서 HMAC 시크릿 키를 불러옵니다.
HMAC_SECRET_KEY_STR = os.getenv("HMAC_SECRET_KEY")

# 위치를 업데이트할 차량의 고유 ID (이전에 등록한 도둑 차량의 ID)
VEHICLE_ID = 123

# 업데이트할 새로운 X, Y 좌표
POSITION_X = 90.001
POSITION_Y = 20.502
# ---------------------------------------------

# --- 💡 수정된 부분: 패킷 구조 정의 ---
# 메시지 타입 (0x13: 위치 정보 브로드캐스트)
MESSAGE_TYPE = 0x13

def generate_location_packet():
    """차량 위치 업데이트(0x13)를 위한 바이너리 패킷을 생성합니다."""

    if not HMAC_SECRET_KEY_STR:
        print("🛑 에러: .env 파일에 HMAC_SECRET_KEY가 없거나 파일이 존재하지 않습니다.")
        return

    HMAC_SECRET_KEY = HMAC_SECRET_KEY_STR.encode('utf-8')

    # 1. 💡 수정된 부분: HMAC을 제외한 앞부분 데이터를 패킹합니다. (총 13바이트)
    # < : Little-endian
    # B : unsigned char (1 byte) - 메시지 타입 ✨
    # I : unsigned int (4 bytes) - 차량 ID
    # f : float (4 bytes) - X 좌표
    # f : float (4 bytes) - Y 좌표
    header_data = struct.pack('<BIff', MESSAGE_TYPE, VEHICLE_ID, POSITION_X, POSITION_Y)

    # 2. 생성된 헤더 데이터를 기반으로 HMAC 인증 코드를 계산합니다.
    hmac_code = hmac.new(HMAC_SECRET_KEY, header_data, hashlib.sha256).digest()[:16]

    # 3. 💡 수정된 부분: 헤더 데이터와 HMAC 코드를 합쳐 최종 패킷을 완성합니다. (총 29바이트)
    full_packet = header_data + hmac_code

    print("✅ 위치 업데이트 패킷 생성 완료!")
    print("-" * 30)
    print(f"  - 메시지 타입: {hex(MESSAGE_TYPE)}")
    print(f"  - 차량 ID: {VEHICLE_ID}")
    print(f"  - 좌표: X={POSITION_X}, Y={POSITION_Y}")
    print(f"  - 최종 패킷 길이: {len(full_packet)} bytes")
    print("-" * 30)
    print("👇 아래 16진수 문자열을 복사해서 Postman에 사용하세요.")
    print(f"\n생성된 패킷 (Hex):")
    print(full_packet.hex())


if __name__ == "__main__":
    generate_location_packet()