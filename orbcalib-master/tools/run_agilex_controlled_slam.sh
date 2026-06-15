#!/usr/bin/env bash
set -euo pipefail

# Run from inside the orbcalib Docker container.
# It starts roscore, runs orbcalib in SLAM mode with ACKs enabled, streams two
# PNG folders, then sends SIGINT so ORB-SLAM saves atlases into the run folder.

REPO_DIR=${REPO_DIR:-/ws/src/orbcalib-master}
DATASET_ROOT=${DATASET_ROOT:-"/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/bigLoopNoTilt"}
CAMERA1_NAME=${CAMERA1_NAME:-back}
CAMERA2_NAME=${CAMERA2_NAME:-front}
CAMERA1_DIR=${CAMERA1_DIR:-"$DATASET_ROOT/$CAMERA1_NAME"}
CAMERA2_DIR=${CAMERA2_DIR:-"$DATASET_ROOT/$CAMERA2_NAME"}
TOPIC1=${TOPIC1:-/cam_back/image}
TOPIC2=${TOPIC2:-/cam_front/image}
FRAME_ID1=${FRAME_ID1:-agilex_back}
FRAME_ID2=${FRAME_ID2:-agilex_front}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CONTROLLED_CONFIG=${CONTROLLED_CONFIG:-"$REPO_DIR/config/sim/calib_agilex_controlled.yaml"}
CAMERA1_CONFIG=${CAMERA1_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA1_NAME}_cam.yaml"}
CAMERA2_CONFIG=${CAMERA2_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA2_NAME}_cam.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build/calib/calib"}
MAX_IN_FLIGHT=${MAX_IN_FLIGHT:-1}
ACK_TIMEOUT_SEC=${ACK_TIMEOUT_SEC:-120}
PAIRING=${PAIRING:-nearest}
MAX_SKEW_SEC=${MAX_SKEW_SEC:-0.05}
HZ=${HZ:-0}
PLAYBACK_RATE=${PLAYBACK_RATE:-0}
START_INDEX=${START_INDEX:-1}
MAX_PAIRS=${MAX_PAIRS:-0}
STOP_CAMERA1_STAMP=${STOP_CAMERA1_STAMP:-0}
STOP_CAMERA2_STAMP=${STOP_CAMERA2_STAMP:-0}
ENCODING=${ENCODING:-rgb8}
USE_VIEWER=${USE_VIEWER:-}
VIEWER_WARMUP_SEC=${VIEWER_WARMUP_SEC:-5}
PAUSE_BEFORE_PLAYBACK=${PAUSE_BEFORE_PLAYBACK:-0}
SKIP_BAD_IMAGES=${SKIP_BAD_IMAGES:-0}
RUN_ID=${RUN_ID:-$(date +"%Y-%m-%d_%H-%M-%S")_${CAMERA1_NAME}_${CAMERA2_NAME}_agilex_controlled}
RESULTS_ROOT=${RESULTS_ROOT:-"$REPO_DIR/results_agilex"}
RUN_DIR=${RUN_DIR:-"$RESULTS_ROOT/$RUN_ID"}
RESULTS_CHMOD=${RESULTS_CHMOD:-a+rwX}
HOST_UID=${HOST_UID:-}
HOST_GID=${HOST_GID:-$HOST_UID}
CAMERA1_CONFIG_EXPLICIT=0
CAMERA2_CONFIG_EXPLICIT=0
TOPIC1_EXPLICIT=0
TOPIC2_EXPLICIT=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --dataset-root PATH     Folder containing front/back/left/right PNG folders.
  --camera1 NAME          Camera 1 folder name. Default: $CAMERA1_NAME
  --camera2 NAME          Camera 2 folder name. Default: $CAMERA2_NAME
  --camera1-dir PATH      Explicit PNG folder for camera 1.
  --camera2-dir PATH      Explicit PNG folder for camera 2.
  --topic1 TOPIC          ROS image topic for camera 1. Default: $TOPIC1
  --topic2 TOPIC          ROS image topic for camera 2. Default: $TOPIC2
  --camera1-config PATH   ORB-SLAM camera config for camera 1.
  --camera2-config PATH   ORB-SLAM camera config for camera 2.
  --run-id NAME           Result folder name under results_agilex.
  --run-dir PATH          Full result folder path.
  --max-in-flight N       Published frame pairs allowed without ACK.
  --ack-timeout-sec SEC   ACK timeout.
  --pairing ordered|nearest
  --max-skew-sec SEC      Used for nearest pairing.
  --hz HZ                 Optional downsample rate. 0 publishes all pairs.
  --playback-rate RATE    0 means ACK-paced as fast as SLAM allows.
  --start-index N         1-based selected pair index.
  --max-pairs N           0 means all pairs.
  --stop-camera1-stamp NS Stop after camera 1 reaches this PNG timestamp.
  --stop-camera2-stamp NS Stop after camera 2 reaches this PNG timestamp.
  --encoding rgb8|bgr8|mono8
  --viewer                Enable ORB-SLAM Pangolin viewer for this run.
  --no-viewer             Disable ORB-SLAM Pangolin viewer for this run.
  --viewer-warmup-sec SEC Seconds to wait before publishing frame 1. Default: $VIEWER_WARMUP_SEC
  --pause-before-playback Wait for Enter before publishing frame 1.
  --skip-bad-images       Skip frame pairs whose PNGs cannot be decoded.
  -h, --help              Show this help.

Environment overrides are also supported.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset-root) DATASET_ROOT="$2"; CAMERA1_DIR="$DATASET_ROOT/$CAMERA1_NAME"; CAMERA2_DIR="$DATASET_ROOT/$CAMERA2_NAME"; shift 2 ;;
    --camera1) CAMERA1_NAME="$2"; CAMERA1_DIR="$DATASET_ROOT/$CAMERA1_NAME"; shift 2 ;;
    --camera2) CAMERA2_NAME="$2"; CAMERA2_DIR="$DATASET_ROOT/$CAMERA2_NAME"; shift 2 ;;
    --camera1-dir) CAMERA1_DIR="$2"; shift 2 ;;
    --camera2-dir) CAMERA2_DIR="$2"; shift 2 ;;
    --topic1) TOPIC1="$2"; TOPIC1_EXPLICIT=1; shift 2 ;;
    --topic2) TOPIC2="$2"; TOPIC2_EXPLICIT=1; shift 2 ;;
    --camera1-config) CAMERA1_CONFIG="$2"; CAMERA1_CONFIG_EXPLICIT=1; shift 2 ;;
    --camera2-config) CAMERA2_CONFIG="$2"; CAMERA2_CONFIG_EXPLICIT=1; shift 2 ;;
    --run-id) RUN_ID="$2"; RUN_DIR="$RESULTS_ROOT/$RUN_ID"; shift 2 ;;
    --run-dir) RUN_DIR="$2"; shift 2 ;;
    --max-in-flight) MAX_IN_FLIGHT="$2"; shift 2 ;;
    --ack-timeout-sec) ACK_TIMEOUT_SEC="$2"; shift 2 ;;
    --pairing) PAIRING="$2"; shift 2 ;;
    --max-skew-sec) MAX_SKEW_SEC="$2"; shift 2 ;;
    --hz) HZ="$2"; shift 2 ;;
    --playback-rate) PLAYBACK_RATE="$2"; shift 2 ;;
    --start-index) START_INDEX="$2"; shift 2 ;;
    --max-pairs) MAX_PAIRS="$2"; shift 2 ;;
    --stop-camera1-stamp) STOP_CAMERA1_STAMP="$2"; shift 2 ;;
    --stop-camera2-stamp) STOP_CAMERA2_STAMP="$2"; shift 2 ;;
    --encoding) ENCODING="$2"; shift 2 ;;
    --viewer) USE_VIEWER=1; shift ;;
    --no-viewer) USE_VIEWER=0; shift ;;
    --viewer-warmup-sec) VIEWER_WARMUP_SEC="$2"; shift 2 ;;
    --pause-before-playback) PAUSE_BEFORE_PLAYBACK=1; shift ;;
    --skip-bad-images) SKIP_BAD_IMAGES=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$CAMERA1_CONFIG_EXPLICIT" != "1" ]]; then
  CAMERA1_CONFIG="$REPO_DIR/config/sim/agilex_${CAMERA1_NAME}_cam.yaml"
fi
if [[ "$CAMERA2_CONFIG_EXPLICIT" != "1" ]]; then
  CAMERA2_CONFIG="$REPO_DIR/config/sim/agilex_${CAMERA2_NAME}_cam.yaml"
fi
if [[ "$TOPIC1_EXPLICIT" != "1" ]]; then
  TOPIC1="/cam_${CAMERA1_NAME}/image"
fi
if [[ "$TOPIC2_EXPLICIT" != "1" ]]; then
  TOPIC2="/cam_${CAMERA2_NAME}/image"
fi
FRAME_ID1="agilex_${CAMERA1_NAME}"
FRAME_ID2="agilex_${CAMERA2_NAME}"

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Missing required file: $1" >&2
    exit 1
  fi
}

require_dir() {
  if [[ ! -d "$1" ]]; then
    echo "Missing required directory: $1" >&2
    echo "If this is the Robot dataset, mount it into the container, e.g. -v /home/civit/Desktop/Dorsa/Robot:/ws/src/Robot" >&2
    exit 1
  fi
}

fix_result_permissions() {
  if [[ ! -d "$RUN_DIR" ]]; then
    return
  fi
  if [[ -n "$HOST_UID" ]]; then
    chown -R "${HOST_UID}:${HOST_GID}" "$RUN_DIR" 2>/dev/null || true
  fi
  if [[ -n "$RESULTS_CHMOD" ]]; then
    chmod -R "$RESULTS_CHMOD" "$RUN_DIR" 2>/dev/null || true
  fi
}

make_slam_camera_config() {
  local src="$1"
  local dst="$2"
  local atlas_prefix="$3"

  awk -v save_line="System.SaveAtlasToFile: \"$atlas_prefix\"" '
    BEGIN { wrote_atlas = 0 }
    /^System\.(LoadAtlasFromFile|SaveAtlasToFile):/ {
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

make_controlled_config() {
  local src="$1"
  local dst="$2"

  awk \
    -v use_viewer="$USE_VIEWER" \
    -v camera1_image="Camera1.Image: \"$TOPIC1\"" \
    -v camera2_image="Camera2.Image: \"$TOPIC2\"" '
    BEGIN {
      wrote_viewer = 0
      wrote_camera1_image = 0
      wrote_camera2_image = 0
    }
    /^UseViewer:/ {
      if (use_viewer != "") {
        print "UseViewer: " use_viewer
      } else {
        print
      }
      wrote_viewer = 1
      next
    }
    /^Camera1\.Image:/ {
      print camera1_image
      wrote_camera1_image = 1
      next
    }
    /^Camera2\.Image:/ {
      print camera2_image
      wrote_camera2_image = 1
      next
    }
    { print }
    END {
      if (use_viewer != "" && !wrote_viewer) {
        print "UseViewer: " use_viewer
      }
      if (!wrote_camera1_image) {
        print camera1_image
      }
      if (!wrote_camera2_image) {
        print camera2_image
      }
    }
  ' "$src" > "$dst"
}

cd "$REPO_DIR"
set +u
source /opt/ros/noetic/setup.bash
set -u

require_dir "$CAMERA1_DIR"
require_dir "$CAMERA2_DIR"
require_file "$VOCAB_PATH"
require_file "$CONTROLLED_CONFIG"
require_file "$CAMERA1_CONFIG"
require_file "$CAMERA2_CONFIG"
require_file "$CALIB_BIN"

mkdir -p "$RUN_DIR/config"
fix_result_permissions
RUN_REL=${RUN_DIR#"$REPO_DIR"/}

make_slam_camera_config "$CAMERA1_CONFIG" "$RUN_DIR/config/${CAMERA1_NAME}_controlled_slam.yaml" "$RUN_REL/${CAMERA1_NAME}_atlas"
make_slam_camera_config "$CAMERA2_CONFIG" "$RUN_DIR/config/${CAMERA2_NAME}_controlled_slam.yaml" "$RUN_REL/${CAMERA2_NAME}_atlas"
make_controlled_config "$CONTROLLED_CONFIG" "$RUN_DIR/config/calib_agilex_controlled.yaml"

cat > "$RUN_DIR/manifest.txt" <<EOF
run_id=$RUN_ID
run_dir=$RUN_DIR
dataset_root=$DATASET_ROOT
camera1_name=$CAMERA1_NAME
camera2_name=$CAMERA2_NAME
camera1_dir=$CAMERA1_DIR
camera2_dir=$CAMERA2_DIR
topic1=$TOPIC1
topic2=$TOPIC2
pairing=$PAIRING
max_skew_sec=$MAX_SKEW_SEC
hz=$HZ
playback_rate=$PLAYBACK_RATE
max_in_flight=$MAX_IN_FLIGHT
ack_timeout_sec=$ACK_TIMEOUT_SEC
start_index=$START_INDEX
max_pairs=$MAX_PAIRS
stop_camera1_stamp=$STOP_CAMERA1_STAMP
stop_camera2_stamp=$STOP_CAMERA2_STAMP
encoding=$ENCODING
use_viewer=${USE_VIEWER:-from_config}
viewer_warmup_sec=$VIEWER_WARMUP_SEC
pause_before_playback=$PAUSE_BEFORE_PLAYBACK
skip_bad_images=$SKIP_BAD_IMAGES
frame_pairs_csv=$RUN_REL/frame_pairs.csv
started_at=$(date -Iseconds)
repo_dir=$REPO_DIR
calib_bin=$CALIB_BIN
controlled_config=$CONTROLLED_CONFIG
camera1_source_config=$CAMERA1_CONFIG
camera2_source_config=$CAMERA2_CONFIG
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
  fix_result_permissions
  exit "$status"
}
trap cleanup EXIT INT TERM

roscore > "$RUN_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 3

export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
"$CALIB_BIN" \
  "$VOCAB_PATH" \
  "$RUN_DIR/config/calib_agilex_controlled.yaml" \
  "$RUN_DIR/config/${CAMERA1_NAME}_controlled_slam.yaml" \
  "$RUN_DIR/config/${CAMERA2_NAME}_controlled_slam.yaml" \
  > "$RUN_DIR/slam.log" 2>&1 &
CALIB_PID=$!

echo "Waiting ${VIEWER_WARMUP_SEC}s before publishing frames..."
sleep "$VIEWER_WARMUP_SEC"
if [[ "$PAUSE_BEFORE_PLAYBACK" == "1" ]]; then
  echo "ORB-SLAM is running. Bring the GUI windows forward, then press Enter to publish frame 1."
  if [[ -r /dev/tty ]]; then
    read -r _ < /dev/tty
  else
    read -r _
  fi
fi

python3 "$REPO_DIR/tools/controlled_png_pair_player.py" \
  --camera1-dir "$CAMERA1_DIR" \
  --camera2-dir "$CAMERA2_DIR" \
  --topic1 "$TOPIC1" \
  --topic2 "$TOPIC2" \
  --frame-id1 "$FRAME_ID1" \
  --frame-id2 "$FRAME_ID2" \
  --ack1 /orbcalib/camera1/processed \
  --ack2 /orbcalib/camera2/processed \
  --pairing "$PAIRING" \
  --max-skew-sec "$MAX_SKEW_SEC" \
  --hz "$HZ" \
  --playback-rate "$PLAYBACK_RATE" \
  --max-in-flight "$MAX_IN_FLIGHT" \
  --timeout-sec "$ACK_TIMEOUT_SEC" \
  --start-index "$START_INDEX" \
  --max-pairs "$MAX_PAIRS" \
  --stop-camera1-stamp "$STOP_CAMERA1_STAMP" \
  --stop-camera2-stamp "$STOP_CAMERA2_STAMP" \
  --encoding "$ENCODING" \
  --frame-map-csv "$RUN_DIR/frame_pairs.csv" \
  $([[ "$SKIP_BAD_IMAGES" == "1" ]] && printf '%s' "--skip-bad-images") \
  --wait-for-subscribers \
  2>&1 | tee "$RUN_DIR/player.log"

echo "Controlled PNG player finished and final frame ACKs were received."
echo "Stopping orbcalib with SIGINT so ORB-SLAM saves atlases..."
kill -INT "$CALIB_PID"
CALIB_STATUS=0
wait "$CALIB_PID" || CALIB_STATUS=$?
CALIB_PID=""

if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -INT "$ROSCORE_PID" 2>/dev/null || true
  wait "$ROSCORE_PID" || true
  ROSCORE_PID=""
fi

{
  echo "finished_at=$(date -Iseconds)"
  echo "orbcalib_exit_status=$CALIB_STATUS"
  echo "atlas_files:"
  ls -lh "$RUN_DIR"/*atlasCamera*.osa 2>/dev/null || true
} >> "$RUN_DIR/manifest.txt"

fix_result_permissions

if [[ "$CALIB_STATUS" -ne 0 ]]; then
  if compgen -G "$RUN_DIR/*atlasCamera*.osa" > /dev/null; then
    echo "orbcalib exited with status $CALIB_STATUS after shutdown, but atlas files exist."
  else
    echo "orbcalib exited with status $CALIB_STATUS and no atlas files were found in $RUN_DIR." >&2
    exit "$CALIB_STATUS"
  fi
fi

echo "Controlled Agilex SLAM run complete."
echo "Outputs:"
ls -lh "$RUN_DIR"
