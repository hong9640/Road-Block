#!/usr/bin/env bash
set -euo pipefail

USER_ID="ssafy_ad_005"

echo "[gear_setter] 1) SyncMode ON (time_step=20, 빠른 반영)"
rosservice call /SyncModeCmd "
request:
  user_id: '${USER_ID}'
  start_sync_mode: true
  time_step: 100
"

echo "[gear_setter] 2) 기어 D(4)로 변경 (frame=0)"
rosservice call /SyncModeSetGear "
request:
  gear: 4
  frame: 0
"

# 약간의 딜레이 없이 바로 OFF 해도 됨
echo "[gear_setter] 3) SyncMode OFF (실시간 모드 복귀)"
rosservice call /SyncModeCmd "
request:
  user_id: '${USER_ID}'
  start_sync_mode: false
  time_step: 0
"

