#!/usr/bin/env bash
set -euo pipefail

USER_ID="ssafy_ad_005"     # SyncModeWaitForTick에서 사용할 user_id
TARGET_SPEED_KMH="30"      # 목표 속도 (km/h 단위)

# km/h → m/s 변환
kmh_to_ms() {
  awk "BEGIN { printf \"%.2f\", $1/3.6 }"
}

TARGET_SPEED_MS=$(kmh_to_ms "${TARGET_SPEED_KMH}")

# 현재 프레임 읽기
get_frame() {
  rostopic echo -n 1 /SyncModeInfo 2>/dev/null | awk '/^frame:/{print $2; exit}'
}

# 기어 D(=4)로 변경
set_gear_d() {
  local frame="$1"
  echo "[auto_drive] 기어 D(4)로 설정 (frame=$frame)"
  rosservice call /SyncModeSetGear "
request:
  gear: 4
  frame: ${frame}
"
}

# 속도 명령 전송 (velocity 모드)
send_velocity_cmd() {
  local frame="$1"
  rosservice call /SyncModeCtrlCmd "
request:
  frame: ${frame}
  command:
    longlCmdType: 1        # 1 = velocity 모드
    accel: 0.0
    brake: 0.0
    steering: 0.0
    velocity: ${TARGET_SPEED_MS}
    acceleration: 0.0
  sensor_capture: false
"
}

# 틱 진행
wait_tick() {
  local frame="$1"
  rosservice call /SyncModeWaitForTick "
request:
  user_id: '${USER_ID}'
  frame: ${frame}
"
}

# === 실행부 ===
echo "[auto_drive] 현재 프레임 동기화 중..."
CUR_FRAME="$(get_frame || true)"
if [[ -z "${CUR_FRAME:-}" ]]; then
  echo "[auto_drive][에러] /SyncModeInfo에서 frame을 읽지 못했습니다."
  exit 1
fi
echo "[auto_drive] 시작 frame=${CUR_FRAME}"

# 1) 최초 1회 기어 D로 변경
set_gear_d "${CUR_FRAME}"

# 2) 같은 frame으로 틱 진행 → frame+1
wait_tick "${CUR_FRAME}" >/dev/null 2>&1 || true

# 3) 새로운 frame 읽기
CUR_FRAME="$(get_frame)"
echo "[auto_drive] 목표 속도: ${TARGET_SPEED_KMH} km/h (${TARGET_SPEED_MS} m/s)"

# 4) 메인 루프: 전진 명령만 반복
trap 'echo; echo "[auto_drive] 중단되었습니다."; exit 0' INT
while true; do
  # 현재 frame에서 속도 명령
  send_velocity_cmd "${CUR_FRAME}" >/dev/null || {
    echo "[auto_drive][에러] CtrlCmd 실패 → 재동기화"
    CUR_FRAME="$(get_frame)"
    continue
  }

  # 같은 frame으로 틱 진행
  wait_tick "${CUR_FRAME}" >/dev/null || {
    echo "[auto_drive][에러] WaitForTick 실패 → 재동기화"
    CUR_FRAME="$(get_frame)"
    continue
  }

  # 다음 frame 갱신
  CUR_FRAME="$(get_frame)"
  sleep 0.02
done

