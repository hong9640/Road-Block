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

# 등록할 도둑 차량의 고유 ID (임의의 정수)
VEHICLE_ID = 9901 

# 등록할 도둑 차량의 이름 (최대 10바이트)
CAR_NAME = "Runner-X"
# ---------------------------------------------

# --- 패킷 구조 정의 ---
# 메시지 타입 (0xA0: 등록 요청)
MESSAGE_TYPE = 0xA0

# 차량 타입 (0: 경찰차, 1: 도둑차)
VEHICLE_TYPE = 1

def generate_registration_packet():
    """도둑 차량 등록을 위한 바이너리 패킷을 생성합니다."""

    # .env 파일에서 키를 제대로 불러왔는지 확인
    if not HMAC_SECRET_KEY_STR:
        print("🛑 에러: .env 파일에 HMAC_SECRET_KEY가 없거나 파일이 존재하지 않습니다.")
        print("스크립트와 같은 위치에 .env 파일을 만들고 키를 설정해주세요.")
        return

    HMAC_SECRET_KEY = HMAC_SECRET_KEY_STR.encode('utf-8')

    # 1. 차량 이름을 UTF-8로 인코딩하고, 10바이트가 되도록 null byte(\x00)로 채웁니다.
    car_name_bytes = CAR_NAME.encode('utf-8').ljust(10, b'\x00')

    # 2. HMAC을 제외한 앞부분 데이터를 패킹합니다. (총 16바이트)
    header_data = struct.pack('<BIB10s', MESSAGE_TYPE, VEHICLE_ID, VEHICLE_TYPE, car_name_bytes)

    # 3. 생성된 헤더 데이터를 기반으로 HMAC 인증 코드를 계산합니다.
    hmac_code = hmac.new(HMAC_SECRET_KEY, header_data, hashlib.sha256).digest()[:16]

    # 4. 헤더 데이터와 HMAC 코드를 합쳐 최종 패킷을 완성합니다. (총 32바이트)
    full_packet = header_data + hmac_code

    print("✅ 패킷 생성 완료!")
    print("-" * 30)
    print(f"  - 차량 ID: {VEHICLE_ID}")
    print(f"  - 차량 이름: {CAR_NAME}")
    print(f"  - 최종 패킷 길이: {len(full_packet)} bytes")
    print("-" * 30)
    print("👇 아래 16진수 문자열을 복사해서 Postman에 사용하세요.")
    print(f"\n생성된 패킷 (Hex):")
    print(full_packet.hex())


if __name__ == "__main__":
    generate_registration_packet()