#!/usr/bin/env bash
set -euo pipefail

# Run from inside the orbcalib Docker container.
# Calibrate using atlases saved by tools/run_finnforest_controlled_slam.sh.

REPO_DIR=${REPO_DIR:-/ws/src/orbcalib-master}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CALIB_CONFIG=${CALIB_CONFIG:-"$REPO_DIR/config/sim/calib_finnforest.yaml"}
C1_CONFIG=${C1_CONFIG:-"$REPO_DIR/config/sim/C1.yaml"}
C4_CONFIG=${C4_CONFIG:-"$REPO_DIR/config/sim/C4.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build/calib/calib"}
RUN_ID=${RUN_ID:-}
RUN_DIR=${RUN_DIR:-}
NMC3D_DIR=${NMC3D_DIR:-}
EXPORT_NMC3D=${EXPORT_NMC3D:-1}
REQUIRE_NMC3D_EXPORT=${REQUIRE_NMC3D_EXPORT:-0}

usage() {
  cat <<EOF
Usage: $(basename "$0") --run-id NAME [options]

Options:
  --run-id NAME           Existing folder under results_finnforest.
  --run-dir PATH          Existing run folder. Overrides --run-id path.
  --nmc3d-dir PATH        NMC3D repo path where atlases should be copied.
  --no-export-nmc3d       Do not copy atlases into NMC3D.
  --require-nmc3d-export  Fail if the NMC3D export cannot be completed.
  -h, --help              Show this help.

Environment overrides are also supported:
  REPO_DIR, RUN_ID, RUN_DIR, VOCAB_PATH, CALIB_CONFIG, C1_CONFIG, C4_CONFIG,
  CALIB_BIN, NMC3D_DIR, EXPORT_NMC3D, REQUIRE_NMC3D_EXPORT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --run-dir)
      RUN_DIR="$2"
      shift 2
      ;;
    --nmc3d-dir)
      NMC3D_DIR="$2"
      shift 2
      ;;
    --no-export-nmc3d)
      EXPORT_NMC3D=0
      shift
      ;;
    --require-nmc3d-export)
      REQUIRE_NMC3D_EXPORT=1
      shift
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
  RUN_DIR="$REPO_DIR/results_finnforest/$RUN_ID"
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

resolve_nmc3d_dir() {
  if [[ -n "$NMC3D_DIR" ]]; then
    return
  fi

  if [[ -d /ws/src/NMC3D ]]; then
    NMC3D_DIR=/ws/src/NMC3D
  elif [[ -d /home/civit/Desktop/Dorsa/NMC3D ]]; then
    NMC3D_DIR=/home/civit/Desktop/Dorsa/NMC3D
  else
    NMC3D_DIR=
  fi
}

export_atlases_to_nmc3d() {
  if [[ "$EXPORT_NMC3D" != "1" ]]; then
    return
  fi

  resolve_nmc3d_dir

  if [[ -z "$NMC3D_DIR" || ! -d "$NMC3D_DIR" ]]; then
    local msg="NMC3D export skipped: NMC3D repo is not visible. Use --nmc3d-dir PATH or mount it into the container."
    if [[ "$REQUIRE_NMC3D_EXPORT" == "1" ]]; then
      echo "$msg" >&2
      exit 1
    fi
    echo "$msg"
    return
  fi

  if [[ -z "$RUN_ID" ]]; then
    RUN_ID=$(basename "$RUN_DIR")
  fi

  local nmc_run_dir="$NMC3D_DIR/results_finnforest/$RUN_ID"
  mkdir -p "$nmc_run_dir"

  echo "Copying atlases to NMC3D run folder:"
  echo "$nmc_run_dir"
  cp -p "$RUN_DIR/c1_atlasCamera 1.osa" "$nmc_run_dir/c1_atlasCamera 1.osa"
  cp -p "$RUN_DIR/c4_atlasCamera 2.osa" "$nmc_run_dir/c4_atlasCamera 2.osa"

  {
    echo "nmc3d_exported_at=$(date -Iseconds)"
    echo "nmc3d_export_source=$RUN_REL"
    echo "nmc3d_export_dest=$nmc_run_dir"
  } >> "$RUN_DIR/manifest.txt"

  {
    echo "run_id=$RUN_ID"
    echo "atlas_source_orbcalib=$RUN_REL"
    echo "atlas_exported_at=$(date -Iseconds)"
    echo "c1_atlas=c1_atlasCamera 1.osa"
    echo "c4_atlas=c4_atlasCamera 2.osa"
  } > "$nmc_run_dir/orbcalib_atlas_export_manifest.txt"
}

require_file "$VOCAB_PATH"
require_file "$CALIB_CONFIG"
require_file "$C1_CONFIG"
require_file "$C4_CONFIG"
require_file "$CALIB_BIN"
require_file "$RUN_DIR/c1_atlasCamera 1.osa"
require_file "$RUN_DIR/c4_atlasCamera 2.osa"

export_atlases_to_nmc3d

mkdir -p "$RUN_DIR/config"

make_load_camera_config "$C1_CONFIG" "$RUN_DIR/config/C1_controlled_calib_load.yaml" "$RUN_REL/c1_atlas"
make_load_camera_config "$C4_CONFIG" "$RUN_DIR/config/C4_controlled_calib_load.yaml" "$RUN_REL/c4_atlas"
cp "$CALIB_CONFIG" "$RUN_DIR/config/calib_finnforest_calib.yaml"

{
  echo
  echo "calibration_started_at=$(date -Iseconds)"
  echo "calibration_config=$CALIB_CONFIG"
  echo "calibration_c1_load_config=$RUN_REL/config/C1_controlled_calib_load.yaml"
  echo "calibration_c4_load_config=$RUN_REL/config/C4_controlled_calib_load.yaml"
} >> "$RUN_DIR/manifest.txt"

echo "Running orbcalib calibration from run folder: $RUN_REL"

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
  "$RUN_DIR/config/calib_finnforest_calib.yaml" \
  "$RUN_DIR/config/C1_controlled_calib_load.yaml" \
  "$RUN_DIR/config/C4_controlled_calib_load.yaml" \
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
