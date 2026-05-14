#!/usr/bin/env bash
# Orchestrate Contest 3 Pick&Place demo using ROS image bridge + ffmpeg recording.

CYCLES=${CYCLES:-3}
OUT=${OUT:-/data/nedo/gazebo_sim/output/contest3_pickplace_1080p.mp4}
WORLD=pickplace_world
REC_DUR_S=${REC_DUR_S:-150}   # max wall-clock for recorder; controller usually faster

mkdir -p /data/nedo/gazebo_sim/output

set +u
source /opt/ros/jazzy/setup.bash
set +e

export DISPLAY=${DISPLAY:-:0}
export GZ_SIM_RESOURCE_PATH=/data/nedo/gazebo_sim/models:${GZ_SIM_RESOURCE_PATH:-}

cleanup() {
  echo "[demo] === final cleanup ==="
  pkill -INT -f "record_video.py" 2>/dev/null
  pkill -INT -f "pickplace_annotator.py" 2>/dev/null
  pkill -INT -f "pickplace_controller.py" 2>/dev/null
  sleep 1
  pkill -INT -f "ros2 launch" 2>/dev/null
  sleep 1
  pkill -KILL -f "gz sim -r" 2>/dev/null
  pkill -KILL -f "record_video.py" 2>/dev/null
  pkill -KILL -f "pickplace_annotator.py" 2>/dev/null
  pkill -KILL -f "pickplace_controller.py" 2>/dev/null
  pkill -KILL -f "ros2 launch" 2>/dev/null
  pkill -KILL -f "robot_state_publisher" 2>/dev/null
  pkill -KILL -f "controller_manager" 2>/dev/null
  pkill -KILL -f "ros_gz_sim" 2>/dev/null
  pkill -KILL -f "parameter_bridge" 2>/dev/null
  pkill -KILL -f "image_bridge" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

echo "[demo] starting ros2 launch..."
cd /tmp
ros2 launch /data/nedo/gazebo_sim/launch/pickplace_contest3.launch.py > /tmp/pp_launch.log 2>&1 &
LAUNCH_PID=$!
echo "[demo] launch pid=$LAUNCH_PID"

echo "[demo] waiting 16s for sim + controllers + bridge..."
sleep 16

# Reset event log
: > /tmp/arm_events.jsonl

# Start the HUD annotator (reads /cinematic, publishes /cinematic_annotated)
echo "[demo] starting pickplace annotator..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/pickplace_annotator.py > /tmp/pp_annot.log 2>&1 &
AN_PID=$!
sleep 2

# Start the image-bridge -> ffmpeg recorder on the ANNOTATED topic
rm -f "$OUT"
echo "[demo] starting recorder -> $OUT (max ${REC_DUR_S}s)..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/record_video.py /cinematic_annotated "$OUT" "$REC_DUR_S" 1920 1080 > /tmp/pp_rec.log 2>&1 &
REC_PID=$!
sleep 3

# Start pickplace controller
echo "[demo] starting controller (cycles=$CYCLES)..."
/usr/bin/python3 /data/nedo/gazebo_sim/scripts/pickplace_controller.py "$CYCLES" > /tmp/pp_controller.log 2>&1 &
CTL_PID=$!

# Wait for controller
WAIT_S=0
while kill -0 "$CTL_PID" 2>/dev/null && [ "$WAIT_S" -lt 280 ]; do
  sleep 2
  WAIT_S=$((WAIT_S+2))
done
echo "[demo] controller phase done (waited ${WAIT_S}s)"

# Give 3s for final motion + bag to settle
sleep 3

# Stop recorder (it will close ffmpeg properly and finalize mp4)
echo "[demo] stopping recorder..."
kill -INT "$REC_PID" 2>/dev/null
# Wait up to 35s for ffmpeg to flush and exit
wait_count=0
while kill -0 "$REC_PID" 2>/dev/null && [ "$wait_count" -lt 35 ]; do
  sleep 1
  wait_count=$((wait_count+1))
done
echo "[demo] recorder closed (waited ${wait_count}s)"

ls -la "$OUT" 2>/dev/null
echo "[demo] done"
