F=$(rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}')
echo "current frame=$F"

# 1~4 테스트 (없는 값은 응답 result:false 이거나 변화 없음)
for g in 1 2 3 4; do
  echo "== try gear=$g at frame=$F =="
  rosservice call /SyncModeSetGear "
request:
  gear: $g
  frame: $F
"
  # 한 틱 진행
  rosservice call /SyncModeWaitForTick "
request:
  user_id: 'ssafy_ad_005'
  frame: $F
"
  # 다음 프레임으로 갱신
  F=$(rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}')
  echo "moved to frame=$F  -> 화면 우측 Gear 표시를 확인하세요."
  sleep 0.5
done
