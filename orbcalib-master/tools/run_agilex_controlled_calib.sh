#!/usr/bin/env bash
set -euo pipefail

# Run from inside the orbcalib Docker container.
# Calibrate using atlases saved by tools/run_agilex_controlled_slam.sh.

REPO_DIR=${REPO_DIR:-/ws/src/orbcalib-master}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CALIB_CONFIG=${CALIB_CONFIG:-"$REPO_DIR/config/sim/calib_agilex.yaml"}
CAMERA1_NAME=${CAMERA1_NAME:-back}
CAMERA2_NAME=${CAMERA2_NAME:-front}
CAMERA1_CONFIG=${CAMERA1_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA1_NAME}_cam.yaml"}
CAMERA2_CONFIG=${CAMERA2_CONFIG:-"$REPO_DIR/config/sim/agilex_${CAMERA2_NAME}_cam.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build/calib/calib"}
RUN_ID=${RUN_ID:-}
RUN_DIR=${RUN_DIR:-}
RESULTS_ROOT=${RESULTS_ROOT:-"$REPO_DIR/results_agilex"}
RESULTS_CHMOD=${RESULTS_CHMOD:-a+rwX}
HOST_UID=${HOST_UID:-}
HOST_GID=${HOST_GID:-$HOST_UID}
NMC3D_DIR=${NMC3D_DIR:-}
EXPORT_NMC3D=${EXPORT_NMC3D:-0}
USE_VIEWER=${USE_VIEWER:-}
CAMERA1_CONFIG_EXPLICIT=0
CAMERA2_CONFIG_EXPLICIT=0

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
  --viewer                Enable ORB-SLAM Pangolin viewer for this run.
  --no-viewer             Disable ORB-SLAM Pangolin viewer for this run.
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
    --camera1-config) CAMERA1_CONFIG="$2"; CAMERA1_CONFIG_EXPLICIT=1; shift 2 ;;
    --camera2-config) CAMERA2_CONFIG="$2"; CAMERA2_CONFIG_EXPLICIT=1; shift 2 ;;
    --nmc3d-dir) NMC3D_DIR="$2"; shift 2 ;;
    --export-nmc3d) EXPORT_NMC3D=1; shift ;;
    --viewer) USE_VIEWER=1; shift ;;
    --no-viewer) USE_VIEWER=0; shift ;;
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

resolve_atlas_prefix() {
  local camera_name="$1"
  local camera_index="$2"
  local legacy_prefix="$RUN_REL/${camera_name}_atlas"
  local agilex_prefix="$RUN_REL/Agilex_${camera_name}_atlas"

  if [[ -f "$RUN_DIR/Agilex_${camera_name}_atlasCamera ${camera_index}.osa" ]]; then
    echo "$agilex_prefix"
    return
  fi

  if [[ -f "$RUN_DIR/${camera_name}_atlasCamera ${camera_index}.osa" ]]; then
    echo "$legacy_prefix"
    return
  fi

  echo "Missing atlas for ${camera_name} camera ${camera_index}. Expected one of:" >&2
  echo "  $RUN_DIR/Agilex_${camera_name}_atlasCamera ${camera_index}.osa" >&2
  echo "  $RUN_DIR/${camera_name}_atlasCamera ${camera_index}.osa" >&2
  exit 1
}

make_load_camera_config() {
  local src="$1"
  local dst="$2"
  local atlas_prefix="$3"

  awk -v load_line="System.LoadAtlasFromFile: \"$atlas_prefix\"" '
    BEGIN { wrote_atlas = 0 }
    /^System\.(LoadAtlasFromFile|SaveAtlasToFile):/ {
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

make_calib_config() {
  local src="$1"
  local dst="$2"

  if [[ -z "$USE_VIEWER" ]]; then
    cp "$src" "$dst"
    return
  fi

  awk -v use_viewer="UseViewer: $USE_VIEWER" '
    BEGIN { wrote_viewer = 0 }
    /^UseViewer:/ {
      print use_viewer
      wrote_viewer = 1
      next
    }
    { print }
    END {
      if (!wrote_viewer) {
        print use_viewer
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
  cp -p "$CAMERA1_ATLAS_FILE" "$nmc_run_dir/$(basename "$CAMERA1_ATLAS_FILE")"
  cp -p "$CAMERA2_ATLAS_FILE" "$nmc_run_dir/$(basename "$CAMERA2_ATLAS_FILE")"
  echo "nmc3d_export_dest=$nmc_run_dir" >> "$RUN_DIR/manifest.txt"
}

require_file "$VOCAB_PATH"
require_file "$CALIB_CONFIG"
require_file "$CAMERA1_CONFIG"
require_file "$CAMERA2_CONFIG"
require_file "$CALIB_BIN"

CAMERA1_ATLAS_PREFIX=$(resolve_atlas_prefix "$CAMERA1_NAME" 1)
CAMERA2_ATLAS_PREFIX=$(resolve_atlas_prefix "$CAMERA2_NAME" 2)
CAMERA1_ATLAS_FILE="$REPO_DIR/${CAMERA1_ATLAS_PREFIX}Camera 1.osa"
CAMERA2_ATLAS_FILE="$REPO_DIR/${CAMERA2_ATLAS_PREFIX}Camera 2.osa"
require_file "$CAMERA1_ATLAS_FILE"
require_file "$CAMERA2_ATLAS_FILE"

export_atlases_to_nmc3d

mkdir -p "$RUN_DIR/config"
fix_result_permissions

make_load_camera_config "$CAMERA1_CONFIG" "$RUN_DIR/config/${CAMERA1_NAME}_controlled_calib_load.yaml" "$CAMERA1_ATLAS_PREFIX"
make_load_camera_config "$CAMERA2_CONFIG" "$RUN_DIR/config/${CAMERA2_NAME}_controlled_calib_load.yaml" "$CAMERA2_ATLAS_PREFIX"
make_calib_config "$CALIB_CONFIG" "$RUN_DIR/config/calib_agilex_calib.yaml"

{
  echo
  echo "calibration_started_at=$(date -Iseconds)"
  echo "calibration_config=$CALIB_CONFIG"
  echo "calibration_camera1_load_config=$RUN_REL/config/${CAMERA1_NAME}_controlled_calib_load.yaml"
  echo "calibration_camera2_load_config=$RUN_REL/config/${CAMERA2_NAME}_controlled_calib_load.yaml"
  echo "calibration_camera1_atlas=$CAMERA1_ATLAS_PREFIX"
  echo "calibration_camera2_atlas=$CAMERA2_ATLAS_PREFIX"
  echo "calibration_use_viewer=${USE_VIEWER:-from_config}"
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
  fix_result_permissions
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

fix_result_permissions

echo "Calibration complete. Log saved to:"
echo "$RUN_DIR/calib.log"
