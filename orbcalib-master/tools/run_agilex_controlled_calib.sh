#!/usr/bin/env bash
set -euo pipefail

# Run from inside the orbcalib Docker container.
# Calibrate using atlases saved by tools/run_agilex_controlled_slam.sh.

REPO_DIR=${REPO_DIR:-/ws/src/orbcalib-master}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CALIB_CONFIG=${CALIB_CONFIG:-"$REPO_DIR/config/sim/calib_agilex.yaml"}
CAMERA1_NAME=${CAMERA1_NAME:-front}
CAMERA2_NAME=${CAMERA2_NAME:-back}
CAMERA1_CONFIG=${CAMERA1_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA1_NAME}_cam.yaml"}
CAMERA2_CONFIG=${CAMERA2_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA2_NAME}_cam.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build/calib/calib"}
RUN_ID=${RUN_ID:-}
RUN_DIR=${RUN_DIR:-}
RESULTS_ROOT=${RESULTS_ROOT:-"$REPO_DIR/results_agilex"}
NMC3D_DIR=${NMC3D_DIR:-}
EXPORT_NMC3D=${EXPORT_NMC3D:-0}

usage() {
  cat <<EOF
Usage: $(basename "$0") --run-id NAME [options]

Options:
  --run-id NAME           Existing folder under results_agilex.
  --run-dir PATH          Existing run folder. Overrides --run-id path.
  --camera1 NAME          Camera 1 atlas prefix name. Default: $CAMERA1_NAME
  --camera2 NAME          Camera 2 atlas prefix name. Default: $CAMERA2_NAME
  --camera1-config PATH   ORB-SLAM camera config for camera 1.
  --camera2-config PATH   ORB-SLAM camera config for camera 2.
  --nmc3d-dir PATH        Optional NMC3D repo path where atlases should be copied.
  --export-nmc3d          Copy atlases into NMC3D/results_agilex/<run_id>.
  -h, --help              Show this help.

Environment overrides are also supported.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) RUN_ID="$2"; RUN_DIR="$RESULTS_ROOT/$RUN_ID"; shift 2 ;;
    --run-dir) RUN_DIR="$2"; shift 2 ;;
    --camera1) CAMERA1_NAME="$2"; shift 2 ;;
    --camera2) CAMERA2_NAME="$2"; shift 2 ;;
    --camera1-config) CAMERA1_CONFIG="$2"; shift 2 ;;
    --camera2-config) CAMERA2_CONFIG="$2"; shift 2 ;;
    --nmc3d-dir) NMC3D_DIR="$2"; shift 2 ;;
    --export-nmc3d) EXPORT_NMC3D=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "$REPO_DIR"
set +u
source /opt/ros/noetic/setup.bash
set -u

if [[ -z "$RUN_DIR" ]]; then
  if [[ -z "$RUN_ID" ]]; then
    echo "Missing --run-id or --run-dir." >&2
    usage >&2
    exit 2
  fi
  RUN_DIR="$RESULTS_ROOT/$RUN_ID"
fi

if [[ ! "$RUN_DIR" = /* ]]; then
  RUN_DIR="$REPO_DIR/$RUN_DIR"
fi

RUN_REL=${RUN_DIR#"$REPO_DIR"/}

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Missing required file: $1" >&2
    exit 1
  fi
}

make_load_camera_config() {
  local src="$1"
  local dst="$2"
  local atlas_prefix="$3"

  awk -v load_line="System.LoadAtlasFromFile: \"$atlas_prefix\"" '
    BEGIN { wrote_atlas = 0 }
    /^System\.(Load|Save)AtlasFromFile:/ {
      if (!wrote_atlas) {
        print load_line
        wrote_atlas = 1
      }
      next
    }
    { print }
    END {
      if (!wrote_atlas) {
        print load_line
      }
    }
  ' "$src" > "$dst"
}

export_atlases_to_nmc3d() {
  if [[ "$EXPORT_NMC3D" != "1" ]]; then
    return
  fi
  if [[ -z "$NMC3D_DIR" ]]; then
    if [[ -d /ws/src/NMC3D ]]; then
      NMC3D_DIR=/ws/src/NMC3D
    else
      echo "NMC3D export skipped: NMC3D repo is not visible. Use --nmc3d-dir PATH." >&2
      return
    fi
  fi
  local nmc_run_dir="$NMC3D_DIR/results_agilex/$(basename "$RUN_DIR")"
  mkdir -p "$nmc_run_dir"
  cp -p "$RUN_DIR/${CAMERA1_NAME}_atlasCamera 1.osa" "$nmc_run_dir/${CAMERA1_NAME}_atlasCamera 1.osa"
  cp -p "$RUN_DIR/${CAMERA2_NAME}_atlasCamera 2.osa" "$nmc_run_dir/${CAMERA2_NAME}_atlasCamera 2.osa"
  echo "nmc3d_export_dest=$nmc_run_dir" >> "$RUN_DIR/manifest.txt"
}

require_file "$VOCAB_PATH"
require_file "$CALIB_CONFIG"
require_file "$CAMERA1_CONFIG"
require_file "$CAMERA2_CONFIG"
require_file "$CALIB_BIN"
require_file "$RUN_DIR/${CAMERA1_NAME}_atlasCamera 1.osa"
require_file "$RUN_DIR/${CAMERA2_NAME}_atlasCamera 2.osa"

export_atlases_to_nmc3d

mkdir -p "$RUN_DIR/config"

make_load_camera_config "$CAMERA1_CONFIG" "$RUN_DIR/config/${CAMERA1_NAME}_controlled_calib_load.yaml" "$RUN_REL/${CAMERA1_NAME}_atlas"
make_load_camera_config "$CAMERA2_CONFIG" "$RUN_DIR/config/${CAMERA2_NAME}_controlled_calib_load.yaml" "$RUN_REL/${CAMERA2_NAME}_atlas"
cp "$CALIB_CONFIG" "$RUN_DIR/config/calib_agilex_calib.yaml"

{
  echo
  echo "calibration_started_at=$(date -Iseconds)"
  echo "calibration_config=$CALIB_CONFIG"
  echo "calibration_camera1_load_config=$RUN_REL/config/${CAMERA1_NAME}_controlled_calib_load.yaml"
  echo "calibration_camera2_load_config=$RUN_REL/config/${CAMERA2_NAME}_controlled_calib_load.yaml"
} >> "$RUN_DIR/manifest.txt"

echo "Running Agilex calibration from run folder: $RUN_REL"

ROSCORE_PID=""

cleanup() {
  local status=$?
  if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "Stopping calibration roscore..."
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roscore > "$RUN_DIR/roscore_calib.log" 2>&1 &
ROSCORE_PID=$!
sleep 3

"$CALIB_BIN" \
  "$VOCAB_PATH" \
  "$RUN_DIR/config/calib_agilex_calib.yaml" \
  "$RUN_DIR/config/${CAMERA1_NAME}_controlled_calib_load.yaml" \
  "$RUN_DIR/config/${CAMERA2_NAME}_controlled_calib_load.yaml" \
  2>&1 | tee "$RUN_DIR/calib.log"

if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -INT "$ROSCORE_PID" 2>/dev/null || true
  wait "$ROSCORE_PID" || true
  ROSCORE_PID=""
fi

{
  echo "calibration_finished_at=$(date -Iseconds)"
  echo "calibration_log=$RUN_REL/calib.log"
  echo "calibration_roscore_log=$RUN_REL/roscore_calib.log"
} >> "$RUN_DIR/manifest.txt"

echo "Calibration complete. Log saved to:"
echo "$RUN_DIR/calib.log"
