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
USE_GLOBAL_MAP_SCALES=${USE_GLOBAL_MAP_SCALES:-}
CAMERA1_GLOBAL_SCALE=${CAMERA1_GLOBAL_SCALE:-}
CAMERA2_GLOBAL_SCALE=${CAMERA2_GLOBAL_SCALE:-}
FIX_SCALE_AFTER_GLOBAL_SCALING=${FIX_SCALE_AFTER_GLOBAL_SCALING:-}
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
  --use-global-map-scales Enable metric map scales during calibration.
  --no-global-map-scales  Disable metric map scales in the generated config.
  --camera1-global-scale S
                          Metric scale for camera 1, in meters / SLAM unit.
  --camera2-global-scale S
                          Metric scale for camera 2, in meters / SLAM unit.
  --fix-scale-after-global-scaling
                          Fix Sim3 scale after applying metric map scales.
  --free-scale-after-global-scaling
                          Let Sim3 scale optimize after applying metric map scales.
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
    --use-global-map-scales) USE_GLOBAL_MAP_SCALES=1; shift ;;
    --no-global-map-scales) USE_GLOBAL_MAP_SCALES=0; shift ;;
    --camera1-global-scale) CAMERA1_GLOBAL_SCALE="$2"; shift 2 ;;
    --camera2-global-scale) CAMERA2_GLOBAL_SCALE="$2"; shift 2 ;;
    --fix-scale-after-global-scaling) FIX_SCALE_AFTER_GLOBAL_SCALING=1; shift ;;
    --free-scale-after-global-scaling) FIX_SCALE_AFTER_GLOBAL_SCALING=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$CAMERA1_GLOBAL_SCALE" || -n "$CAMERA2_GLOBAL_SCALE" ]]; then
  USE_GLOBAL_MAP_SCALES=${USE_GLOBAL_MAP_SCALES:-1}
fi
if [[ "$USE_GLOBAL_MAP_SCALES" == "1" ]]; then
  if [[ -z "$CAMERA1_GLOBAL_SCALE" || -z "$CAMERA2_GLOBAL_SCALE" ]]; then
    echo "--camera1-global-scale and --camera2-global-scale are required with --use-global-map-scales." >&2
    exit 2
  fi
  FIX_SCALE_AFTER_GLOBAL_SCALING=${FIX_SCALE_AFTER_GLOBAL_SCALING:-0}
fi

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

  awk \
    -v have_viewer="$USE_VIEWER" \
    -v have_scales="$USE_GLOBAL_MAP_SCALES" \
    -v use_scales="$USE_GLOBAL_MAP_SCALES" \
    -v camera1_scale="$CAMERA1_GLOBAL_SCALE" \
    -v camera2_scale="$CAMERA2_GLOBAL_SCALE" \
    -v fix_scale="$FIX_SCALE_AFTER_GLOBAL_SCALING" '
    BEGIN { wrote_viewer = 0 }
    /^UseViewer:/ && have_viewer != "" {
      print "UseViewer: " have_viewer
      wrote_viewer = 1
      next
    }
    have_scales != "" && /^Calibration\.(UseGlobalMapScales|Camera1GlobalScale|Camera2GlobalScale|FixScaleAfterGlobalScaling):/ {
      next
    }
    { print }
    END {
      if (have_viewer != "" && !wrote_viewer) {
        print "UseViewer: " have_viewer
      }
      if (have_scales != "") {
        print ""
        print "# Metric map scale settings generated by run_agilex_controlled_calib.sh."
        print "Calibration.UseGlobalMapScales: " use_scales
        if (use_scales == "1") {
          print "Calibration.Camera1GlobalScale: " camera1_scale
          print "Calibration.Camera2GlobalScale: " camera2_scale
          print "Calibration.FixScaleAfterGlobalScaling: " fix_scale
        }
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
if [[ "$USE_GLOBAL_MAP_SCALES" == "1" ]]; then
  mkdir -p "$RUN_DIR/ground_scale"
  CALIB_CONFIG_OUTPUT="$RUN_DIR/ground_scale/calib_agilex_scaled.yaml"
  CALIB_CONFIG_OUTPUT_REL=${CALIB_CONFIG_OUTPUT#"$REPO_DIR"/}
  KEYFRAME_MATCHES_CSV="$RUN_DIR/ground_scale/calib_keyframe_matches_scaled.csv"
  CALIB_LOG="$RUN_DIR/ground_scale/calib_scaled.log"
  ROSCORE_CALIB_LOG="$RUN_DIR/ground_scale/roscore_calib_scaled.log"
else
  CALIB_CONFIG_OUTPUT="$RUN_DIR/config/calib_agilex_calib.yaml"
  CALIB_CONFIG_OUTPUT_REL=${CALIB_CONFIG_OUTPUT#"$REPO_DIR"/}
  KEYFRAME_MATCHES_CSV="$RUN_DIR/calib_keyframe_matches.csv"
  CALIB_LOG="$RUN_DIR/calib.log"
  ROSCORE_CALIB_LOG="$RUN_DIR/roscore_calib.log"
fi
fix_result_permissions

make_load_camera_config "$CAMERA1_CONFIG" "$RUN_DIR/config/${CAMERA1_NAME}_controlled_calib_load.yaml" "$CAMERA1_ATLAS_PREFIX"
make_load_camera_config "$CAMERA2_CONFIG" "$RUN_DIR/config/${CAMERA2_NAME}_controlled_calib_load.yaml" "$CAMERA2_ATLAS_PREFIX"
make_calib_config "$CALIB_CONFIG" "$CALIB_CONFIG_OUTPUT"

{
  echo
  echo "calibration_started_at=$(date -Iseconds)"
  echo "calibration_config=$CALIB_CONFIG"
  echo "calibration_generated_config=$CALIB_CONFIG_OUTPUT_REL"
  echo "calibration_camera1_load_config=$RUN_REL/config/${CAMERA1_NAME}_controlled_calib_load.yaml"
  echo "calibration_camera2_load_config=$RUN_REL/config/${CAMERA2_NAME}_controlled_calib_load.yaml"
  echo "calibration_camera1_atlas=$CAMERA1_ATLAS_PREFIX"
  echo "calibration_camera2_atlas=$CAMERA2_ATLAS_PREFIX"
  echo "calibration_use_viewer=${USE_VIEWER:-from_config}"
  echo "calibration_use_global_map_scales=${USE_GLOBAL_MAP_SCALES:-from_config}"
  echo "calibration_camera1_global_scale=${CAMERA1_GLOBAL_SCALE:-from_config}"
  echo "calibration_camera2_global_scale=${CAMERA2_GLOBAL_SCALE:-from_config}"
  echo "calibration_fix_scale_after_global_scaling=${FIX_SCALE_AFTER_GLOBAL_SCALING:-from_config}"
  echo "calibration_keyframe_matches_csv=${KEYFRAME_MATCHES_CSV#"$REPO_DIR"/}"
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

roscore > "$ROSCORE_CALIB_LOG" 2>&1 &
ROSCORE_PID=$!
sleep 3

CALIB_KEYFRAME_MATCHES_CSV="$KEYFRAME_MATCHES_CSV" "$CALIB_BIN" \
  "$VOCAB_PATH" \
  "$CALIB_CONFIG_OUTPUT" \
  "$RUN_DIR/config/${CAMERA1_NAME}_controlled_calib_load.yaml" \
  "$RUN_DIR/config/${CAMERA2_NAME}_controlled_calib_load.yaml" \
  2>&1 | tee "$CALIB_LOG"

if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -INT "$ROSCORE_PID" 2>/dev/null || true
  wait "$ROSCORE_PID" || true
  ROSCORE_PID=""
fi

{
  echo "calibration_finished_at=$(date -Iseconds)"
  echo "calibration_log=${CALIB_LOG#"$REPO_DIR"/}"
  echo "calibration_roscore_log=${ROSCORE_CALIB_LOG#"$REPO_DIR"/}"
} >> "$RUN_DIR/manifest.txt"

fix_result_permissions

echo "Calibration complete. Log saved to:"
echo "$CALIB_LOG"
