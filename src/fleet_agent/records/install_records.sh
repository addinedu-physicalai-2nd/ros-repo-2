#!/usr/bin/env bash
# 레포 기록(json)을 로봇 홈(~/)에 깐다. 로봇은 역할이 고정이 아니라서 모든 로봇이 전부 갖는다.
#   bash ~/ros-repo-2/src/fleet_agent/records/install_records.sh          # 없는 파일만 깜 (로봇에서 다시 찍은 건 안 건드림)
#   bash ~/ros-repo-2/src/fleet_agent/records/install_records.sh --force  # 레포 값으로 덮어씀 (원래 파일은 ~/records_backup_<시각>/)
# 카메라 보정(dock_cam.json)은 로봇마다 다름 -> cam/<hostname>.json에서 이 로봇 것만 (없을 때만 깜)
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FORCE=0; [ "$1" = "--force" ] && FORCE=1
BK="$HOME/records_backup_$(date +%Y%m%d_%H%M%S)"
n_new=0; n_skip=0; n_over=0
for f in "$DIR"/*.json; do
  b="$(basename "$f")"; t="$HOME/$b"
  if [ -e "$t" ]; then
    if cmp -s "$f" "$t"; then n_skip=$((n_skip+1)); continue; fi
    if [ $FORCE -eq 1 ]; then mkdir -p "$BK"; cp -p "$t" "$BK/"; cp "$f" "$t"; n_over=$((n_over+1)); echo "덮어씀 $b (원래 것: $BK/$b)"
    else n_skip=$((n_skip+1)); echo "그대로 둠 $b (로봇 값이 다름, 레포 값으로 하려면 --force)"; fi
  else cp "$f" "$t"; n_new=$((n_new+1)); echo "새로 $b"; fi
done
echo "끝: 새로 $n_new, 덮어씀 $n_over, 그대로 $n_skip"
# 카메라 보정: 로봇마다 다름 -> cam/<hostname>.json (이 로봇 것만). 이미 있으면 안 건드림
CAM="$DIR/cam/$(hostname).json"
if [ -e "$HOME/dock_cam.json" ]; then
  if [ -e "$CAM" ] && ! cmp -s "$CAM" "$HOME/dock_cam.json"; then
    echo "카메라 보정: ~/dock_cam.json이 레포 cam/$(hostname).json과 다름 (로봇 값 그대로 씀. 새로 보정했으면 레포에 올려 주세요)"
  fi
elif [ -e "$CAM" ]; then
  cp "$CAM" "$HOME/dock_cam.json"; echo "카메라 보정: cam/$(hostname).json -> ~/dock_cam.json"
else
  echo "!! 카메라 보정 없음 ($(hostname)) -> ros2 run fleet_agent marker_dock --calibrate 한 번, 그다음 ~/dock_cam.json을 레포 cam/$(hostname).json으로"
fi
