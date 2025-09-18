#!/usr/bin/env bash
set -euo pipefail
USER_ID="ssafy_ad_007"

rosservice call /SyncModeCmd "request: {user_id: '${USER_ID}', start_sync_mode: true, time_step: 20}"

F=$(rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}')
rosservice call /SyncModeSetGear "request: {gear: 4, frame: $F}"

# 커밋용 1틱
rosservice call /SyncModeWaitForTick "request: {user_id: '${USER_ID}', frame: $F}"

rosservice call /SyncModeCmd "request: {user_id: '${USER_ID}', start_sync_mode: false, time_step: 0}"
echo "[done] gear=D(4) set and syncmode OFF"

