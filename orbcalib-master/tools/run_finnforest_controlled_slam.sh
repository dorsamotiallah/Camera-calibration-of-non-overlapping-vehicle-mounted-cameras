#!/usr/bin/env bash
set -euo pipefail

# Run from inside the orbcalib Docker container.
# It starts roscore, runs orbcalib in SLAM mode with ACKs enabled, plays the
# FinnForest bag with backpressure, then sends SIGINT so ORB-SLAM saves atlases.

REPO_DIR=${REPO_DIR:-/ws/src/orbcalib-master}
BAG_PATH=${BAG_PATH:-"$REPO_DIR/data/S01_C1_C4_40Hz.bag"}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CONTROLLED_CONFIG=${CONTROLLED_CONFIG:-"$REPO_DIR/config/sim/calib_finnforest_controlled.yaml"}
C1_CONFIG=${C1_CONFIG:-"$REPO_DIR/config/sim/C1.yaml"}
C4_CONFIG=${C4_CONFIG:-"$REPO_DIR/config/sim/C4.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build/calib/calib"}
MAX_IN_FLIGHT=${MAX_IN_FLIGHT:-1}
ACK_TIMEOUT_SEC=${ACK_TIMEOUT_SEC:-120}
RUN_ID=${RUN_ID:-$(date +"%Y-%m-%d_%H-%M-%S")_c1_c4_controlled}
RUN_DIR=${RUN_DIR:-"$REPO_DIR/results_finnforest/$RUN_ID"}

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --bag PATH              ROS1 bag path. Default: $BAG_PATH
  --max-in-flight N       Published frame pairs allowed without ACK. Default: $MAX_IN_FLIGHT
  --ack-timeout-sec SEC   ACK timeout. Default: $ACK_TIMEOUT_SEC
  --run-id NAME           Result folder name under results_finnforest.
  -h, --help              Show this help.

Environment overrides are also supported:
  BAG_PATH, MAX_IN_FLIGHT, ACK_TIMEOUT_SEC, RUN_ID, RUN_DIR
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --max-in-flight)
      MAX_IN_FLIGHT="$2"
      shift 2
      ;;
    --ack-timeout-sec)
      ACK_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --run-id)
      RUN_ID="$2"
      RUN_DIR="$REPO_DIR/results_finnforest/$RUN_ID"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Missing required file: $1" >&2
    exit 1
  fi
}

make_slam_camera_config() {
  local src="$1"
  local dst="$2"
  local atlas_prefix="$3"

  awk -v save_line="System.SaveAtlasToFile: \"$atlas_prefix\"" '
    BEGIN { wrote_atlas = 0 }
    /^System\.(Load|Save)AtlasFromFile:/ {
      if (!wrote_atlas) {
        print save_line
        wrote_atlas = 1
      }
      next
    }
    { print }
    END {
      if (!wrote_atlas) {
        print save_line
      }
    }
  ' "$src" > "$dst"
}

cd "$REPO_DIR"
# ROS setup scripts may reference variables such as ROS_MASTER_URI before they
# are set. Temporarily relax nounset only while sourcing ROS.
set +u
source /opt/ros/noetic/setup.bash
set -u

require_file "$BAG_PATH"
require_file "$VOCAB_PATH"
require_file "$CONTROLLED_CONFIG"
require_file "$C1_CONFIG"
require_file "$C4_CONFIG"
require_file "$CALIB_BIN"

mkdir -p "$RUN_DIR"
mkdir -p "$RUN_DIR/config"

make_slam_camera_config "$C1_CONFIG" "$RUN_DIR/config/C1_controlled_slam.yaml" "results_finnforest/$RUN_ID/c1_atlas"
make_slam_camera_config "$C4_CONFIG" "$RUN_DIR/config/C4_controlled_slam.yaml" "results_finnforest/$RUN_ID/c4_atlas"
cp "$CONTROLLED_CONFIG" "$RUN_DIR/config/calib_finnforest_controlled.yaml"

cat > "$RUN_DIR/manifest.txt" <<EOF
run_id=$RUN_ID
run_dir=$RUN_DIR
bag_path=$BAG_PATH
max_in_flight=$MAX_IN_FLIGHT
ack_timeout_sec=$ACK_TIMEOUT_SEC
started_at=$(date -Iseconds)
repo_dir=$REPO_DIR
calib_bin=$CALIB_BIN
controlled_config=$CONTROLLED_CONFIG
c1_source_config=$C1_CONFIG
c4_source_config=$C4_CONFIG
EOF

echo "Result folder: $RUN_DIR"

ROSCORE_PID=""
CALIB_PID=""

cleanup() {
  local status=$?
  if [[ -n "${CALIB_PID}" ]] && kill -0 "$CALIB_PID" 2>/dev/null; then
    echo "Stopping orbcalib with SIGINT..."
    kill -INT "$CALIB_PID" 2>/dev/null || true
    wait "$CALIB_PID" || true
  fi
  if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "Stopping roscore..."
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roscore > "$RUN_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 3

export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
"$CALIB_BIN" \
  "$VOCAB_PATH" \
  "$RUN_DIR/config/calib_finnforest_controlled.yaml" \
  "$RUN_DIR/config/C1_controlled_slam.yaml" \
  "$RUN_DIR/config/C4_controlled_slam.yaml" \
  > "$RUN_DIR/slam.log" 2>&1 &
CALIB_PID=$!

sleep 5

python3 "$REPO_DIR/tools/controlled_finnforest_bag_player.py" \
  --bag "$BAG_PATH" \
  --topic-c1 /cam_c1/image \
  --topic-c4 /cam_c4/image \
  --ack-c1 /orbcalib/camera1/processed \
  --ack-c4 /orbcalib/camera2/processed \
  --max-in-flight "$MAX_IN_FLIGHT" \
  --timeout-sec "$ACK_TIMEOUT_SEC" \
  --wait-for-subscribers \
  2>&1 | tee "$RUN_DIR/player.log"

echo "Controlled player finished and final frame ACKs were received."
echo "Stopping orbcalib with SIGINT so ORB-SLAM saves atlases..."
kill -INT "$CALIB_PID"
wait "$CALIB_PID"
CALIB_PID=""

if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -INT "$ROSCORE_PID" 2>/dev/null || true
  wait "$ROSCORE_PID" || true
  ROSCORE_PID=""
fi

{
  echo "finished_at=$(date -Iseconds)"
  echo "atlas_files:"
  ls -lh "$RUN_DIR"/*atlasCamera*.osa 2>/dev/null || true
} >> "$RUN_DIR/manifest.txt"

echo "Controlled SLAM run complete."
echo "Outputs:"
ls -lh "$RUN_DIR"
