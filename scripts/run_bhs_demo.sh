#!/usr/bin/env bash
# Contest1 BHS demo: gz sim + bag spawner + annotator + recorder.

DUR_S=${DUR_S:-70}
OUT=${OUT:-/data/nedo/gazebo_sim/output/contest1_bhs_1080p.mp4}
WORLD_SDF=/data/nedo/gazebo_sim/worlds/bhs_contest1.sdf

mkdir -p /data/nedo/gazebo_sim/output

set +u
source /opt/ros/jazzy/setup.bash
set +e

export DISPLAY=${DISPLAY:-:0}
export GZ_SIM_RESOURCE_PATH=/data/nedo/gazebo_sim/models:${GZ_SIM_RESOURCE_PATH:-}

cleanup() {
  echo "[demo] cleanup"
  pkill -INT -f "bhs_annotator.py" 2>/dev/null
  pkill -INT -f "record_video.py" 2>/dev/null
  pkill -INT -f "bag_spawner.py" 2>/dev/null
  sleep 1
  pkill -KILL -f "gz sim -r" 2>/dev/null
  pkill -KILL -f "bhs_annotator.py" 2>/dev/null
  pkill -KILL -f "record_video.py" 2>/dev/null
  pkill -KILL -f "bag_spawner.py" 2>/dev/null
  pkill -KILL -f "image_bridge" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

# Reset events file
: > /tmp/bag_events.jsonl

cd /tmp
echo "[demo] gz sim..."
gz sim -r -s -v 2 "$WORLD_SDF" > /tmp/bhs_gz.log 2>&1 &
GZ_PID=$!
sleep 5

echo "[demo] image bridge..."
ros2 run ros_gz_image image_bridge /cinematic > /tmp/bhs_bridge.log 2>&1 &
BR_PID=$!
sleep 2

echo "[demo] BHS annotator (HUD overlay)..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/bhs_annotator.py > /tmp/bhs_annot.log 2>&1 &
AN_PID=$!
sleep 2

echo "[demo] bag spawner..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/bag_spawner.py > /tmp/bhs_spawn.log 2>&1 &
SP_PID=$!
sleep 1

echo "[demo] recorder ($DUR_S s -> $OUT)..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/record_video.py /cinematic_annotated "$OUT" "$DUR_S" 1920 1080 > /tmp/bhs_rec.log 2>&1 &
REC_PID=$!

# Wait for recorder to finish (it will exit by itself when its duration elapses)
WAIT_S=0
while kill -0 "$REC_PID" 2>/dev/null && [ "$WAIT_S" -lt $((DUR_S + 30)) ]; do
  sleep 2
  WAIT_S=$((WAIT_S+2))
done
echo "[demo] recorder finished (waited ${WAIT_S}s)"

ls -la "$OUT" 2>/dev/null
echo "[demo] done"
