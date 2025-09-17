# create_dummy_event.py

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


EVENT_TYPE = "CATCH"  

# 잡은 경찰차의 고유 ID (요청하신 값)
CATCHER_ID = 123 

# 잡힌 도둑 차량의 고유 ID (이전에 등록한 도둑 차량 ID로 가정)
RUNNER_ID = 9901
# ---------------------------------------------

def generate_catch_packet():
    """검거 성공(0xFE) 또는 실패(0xFD) 이벤트를 위한 바이너리 패킷을 생성합니다."""

    if not HMAC_SECRET_KEY_STR:
        print("🛑 에러: .env 파일에 HMAC_SECRET_KEY가 없거나 파일이 존재하지 않습니다.")
        return

    HMAC_SECRET_KEY = HMAC_SECRET_KEY_STR.encode('utf-8')

    # 💡 수정된 부분: EVENT_TYPE 설정에 따라 메시지 타입을 결정
    if EVENT_TYPE == "CATCH":
        MESSAGE_TYPE = 0xFE
        event_name = "성공"
    elif EVENT_TYPE == "FAILED":
        MESSAGE_TYPE = 0xFD
        event_name = "실패"
    else:
        print(f"🛑 에러: EVENT_TYPE은 'success' 또는 'failure'여야 합니다. (현재 값: '{EVENT_TYPE}')")
        return

    # 1. HMAC을 제외한 앞부분 데이터를 패킹합니다. (총 9바이트)
    # < : Little-endian
    # B : unsigned char (1 byte) - 메시지 타입
    # I : unsigned int (4 bytes) - 경찰차 ID
    # I : unsigned int (4 bytes) - 도둑차 ID
    header_data = struct.pack('<BII', MESSAGE_TYPE, CATCHER_ID, RUNNER_ID)

    # 2. 생성된 헤더 데이터를 기반으로 HMAC 인증 코드를 계산합니다.
    hmac_code = hmac.new(HMAC_SECRET_KEY, header_data, hashlib.sha256).digest()[:16]

    # 3. 헤더 데이터와 HMAC 코드를 합쳐 최종 패킷을 완성합니다. (총 25바이트)
    full_packet = header_data + hmac_code

    print(f"✅ 검거 {event_name} 패킷 생성 완료!")
    print("-" * 30)
    print(f"  - 메시지 타입: {hex(MESSAGE_TYPE)}")
    print(f"  - Catcher ID: {CATCHER_ID}")
    print(f"  - Runner ID: {RUNNER_ID}")
    print(f"  - 최종 패킷 길이: {len(full_packet)} bytes")
    print("-" * 30)
    print("👇 아래 16진수 문자열을 복사해서 Postman에 사용하세요.")
    print(f"\n생성된 패킷 (Hex):")
    print(full_packet.hex())


if __name__ == "__main__":
    generate_catch_packet()