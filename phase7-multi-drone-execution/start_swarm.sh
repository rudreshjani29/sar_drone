#!/usr/bin/env bash
# Start N headless PX4 SITL instances sharing ONE Gazebo world (macOS/Linux).
# Usage: ./start_swarm.sh [N=3] [PX4_DIR=~/PX4-Autopilot]
# Commands follow the PX4 multi-vehicle Gazebo docs: first instance starts the
# Gazebo server; later ones use PX4_GZ_STANDALONE=1 and a distinct pose and -i.
set -u
N=${1:-3}
PX4_DIR=${2:-$HOME/PX4-Autopilot}
LOGDIR="$HOME/px4_swarm_logs"
export GZ_IP=127.0.0.1

cd "$PX4_DIR" || { echo "PX4 dir not found: $PX4_DIR"; exit 1; }
PX4_BIN=./build/px4_sitl_default/bin/px4
if [ ! -x "$PX4_BIN" ]; then
  echo "PX4 not built yet. Run once:  HEADLESS=1 make px4_sitl gz_x500   (Ctrl+C after it starts), then retry."
  exit 1
fi
mkdir -p "$LOGDIR"

echo "Starting instance 1 (also starts Gazebo, headless)..."
HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  nohup "$PX4_BIN" -i 1 -d > "$LOGDIR/instance_1.log" 2>&1 < /dev/null &
echo "Waiting 30 s for Gazebo world to come up..."
sleep 30

for i in $(seq 2 "$N"); do
  Y=$(( (i - 1) * 3 ))      # 3 m apart along y so models don't overlap
  echo "Starting instance $i at pose 0,$Y ..."
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,$Y" PX4_SIM_MODEL=gz_x500 \
    nohup "$PX4_BIN" -i "$i" -d > "$LOGDIR/instance_$i.log" 2>&1 < /dev/null &
  sleep 10
done

echo "Started $N instances. Logs: $LOGDIR/instance_*.log"
echo "Check:  tail -n 20 $LOGDIR/instance_2.log"
echo "Stop with ./stop_swarm.sh"
