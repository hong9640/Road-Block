#!/usr/bin/env bash
set -euo pipefail
USER_ID="ssafy_ad_005"

# 1) SyncMode ON (자동 틱 20ms로 살짝만 켬)
rosservice call /SyncModeCmd "request: {user_id: '${USER_ID}', start_sync_mode: true, time_step: 20}"

# 2) 현재 프레임에서 기어 D 전송 (대기/틱 호출 없이)
F=$(rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}')
rosservice call /SyncModeSetGear "request: {gear: 4, frame: $F}"

# (선택) 아주 짧은 여유 (네트워크 큐 플러시 용): sleep 0.02

# 3) 바로 SyncMode OFF (실시간 모드 복귀)
rosservice call /SyncModeCmd "request: {user_id: '${USER_ID}', start_sync_mode: false, time_step: 0}"

