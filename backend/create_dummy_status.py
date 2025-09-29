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

# 상태를 업데이트할 경찰차의 고유 ID (DB에 존재하는 경찰차 ID)
VEHICLE_ID = 9119

# 업데이트할 상태 값
FUEL = 60        # 현재 연료량 (0-100)
COLLISION_COUNT = 2 # 누적 충돌 횟수
STATUS_ENUM = 1     # 차량 상태 (0: NORMAL, 1: HALF_DESTROYED, 2: COMPLETE_DESTROYED)
# ---------------------------------------------

# --- 패킷 구조 정의 ---
# 메시지 타입 (0x12: 상태 업데이트 요청)
MESSAGE_TYPE = 0x12

def generate_status_update_packet():
    """차량 상태 업데이트를 위한 바이너리 패킷을 생성합니다."""

    if not HMAC_SECRET_KEY_STR:
        print("🛑 에러: .env 파일에 HMAC_SECRET_KEY가 없거나 파일이 존재하지 않습니다.")
        return

    HMAC_SECRET_KEY = HMAC_SECRET_KEY_STR.encode('utf-8')

    # 1. HMAC을 제외한 앞부분 데이터를 패킹합니다. (총 8바이트)
    # < : Little-endian
    # B : unsigned char (1 byte) - 메시지 타입
    # I : unsigned int (4 bytes) - 차량 ID
    # B : unsigned char (1 byte) - 연료량
    # B : unsigned char (1 byte) - 충돌 횟수
    # B : unsigned char (1 byte) - 상태 Enum
    header_data = struct.pack('<BIBBB', MESSAGE_TYPE, VEHICLE_ID, FUEL, COLLISION_COUNT, STATUS_ENUM)

    # 2. 생성된 헤더 데이터를 기반으로 HMAC 인증 코드를 계산합니다.
    hmac_code = hmac.new(HMAC_SECRET_KEY, header_data, hashlib.sha256).digest()[:16]

    # 3. 헤더 데이터와 HMAC 코드를 합쳐 최종 패킷을 완성합니다. (총 24바이트)
    full_packet = header_data + hmac_code

    print("✅ 상태 업데이트 패킷 생성 완료!")
    print("-" * 30)
    print(f"  - 차량 ID: {VEHICLE_ID}")
    print(f"  - 연료량: {FUEL}, 충돌 횟수: {COLLISION_COUNT}, 상태: {STATUS_ENUM}")
    print(f"  - 최종 패킷 길이: {len(full_packet)} bytes")
    print("-" * 30)
    print("👇 아래 16진수 문자열을 복사해서 Postman에 사용하세요.")
    print(f"\n생성된 패킷 (Hex):")
    print(full_packet.hex())


if __name__ == "__main__":
    generate_status_update_packet()