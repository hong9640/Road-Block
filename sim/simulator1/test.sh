#!/usr/bin/env bash
set -euo pipefail
USER_ID="ssafy_ad_005"

echo "[1] SyncMode 시작 (자동 tick 20ms)"
rosservice call /SyncModeCmd "request:
  user_id: '${USER_ID}'
  start_sync_mode: true
  time_step: 0
"

F=$(rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}')

echo "[3] 기어 D(4)로 설정"
rosservice call /SyncModeSetGear "request:
  gear: 4
  frame: $F
"

rosservice call /SyncModeWaitForTick "
request:
  user_id: 'ssafy_ad_005'
  frame: $F
"

echo "[gear_setter] 3) SyncMode 종료"
rosservice call /SyncModeCmd "
request:
  user_id: '${USER_ID}'
  start_sync_mode: false
  time_step: 20
"
