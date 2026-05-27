#!/usr/bin/env bash
set -euo pipefail

# Run from inside the NMC3D Docker container.
# Calibrate with NMC3D using atlases copied from an orbcalib controlled run.

REPO_DIR=${REPO_DIR:-/ws/src/NMC3D}
VOCAB_PATH=${VOCAB_PATH:-"$REPO_DIR/Vocabulary/ORBvoc.txt"}
CALIB_CONFIG=${CALIB_CONFIG:-"$REPO_DIR/config/sim/calib_finnforest.yaml"}
C1_CONFIG=${C1_CONFIG:-"$REPO_DIR/config/sim/C1.yaml"}
C4_CONFIG=${C4_CONFIG:-"$REPO_DIR/config/sim/C4.yaml"}
CALIB_BIN=${CALIB_BIN:-"$REPO_DIR/build_nmc_docker/calib/calib"}
BUILD_DIR=${BUILD_DIR:-"$REPO_DIR/build_nmc_docker"}
RUN_ID=${RUN_ID:-}
RUN_DIR=${RUN_DIR:-}
SKIP_BUILD=${SKIP_BUILD:-0}

usage() {
  cat <<EOF
Usage: $(basename "$0") --run-id NAME [options]

Options:
  --run-id NAME     Existing folder under NMC3D/results_finnforest.
  --run-dir PATH    Existing run folder. Overrides --run-id path.
  --skip-build      Do not run cmake configure/build before calibration.
  -h, --help        Show this help.

Environment overrides are also supported:
  REPO_DIR, RUN_ID, RUN_DIR, VOCAB_PATH, CALIB_CONFIG, C1_CONFIG, C4_CONFIG,
  CALIB_BIN, BUILD_DIR, SKIP_BUILD
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
    --skip-build)
      SKIP_BUILD=1
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

if [[ -z "$RUN_ID" ]]; then
  RUN_ID=$(basename "$RUN_DIR")
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

require_file "$VOCAB_PATH"
require_file "$CALIB_CONFIG"
require_file "$C1_CONFIG"
require_file "$C4_CONFIG"
require_file "$RUN_DIR/c1_atlasCamera 1.osa"
require_file "$RUN_DIR/c4_atlasCamera 2.osa"

mkdir -p "$RUN_DIR/config"

make_load_camera_config "$C1_CONFIG" "$RUN_DIR/config/C1_nmc3d_calib_load.yaml" "$RUN_REL/c1_atlas"
make_load_camera_config "$C4_CONFIG" "$RUN_DIR/config/C4_nmc3d_calib_load.yaml" "$RUN_REL/c4_atlas"
cp "$CALIB_CONFIG" "$RUN_DIR/config/calib_finnforest_nmc3d.yaml"

if [[ "$SKIP_BUILD" != "1" ]]; then
  echo "Building NMC3D calib target..."
  cmake -S "$REPO_DIR" -B "$BUILD_DIR"
  cmake --build "$BUILD_DIR" --target calib -j"$(nproc)"
fi

require_file "$CALIB_BIN"

{
  echo
  echo "nmc3d_calibration_started_at=$(date -Iseconds)"
  echo "nmc3d_calibration_config=$RUN_REL/config/calib_finnforest_nmc3d.yaml"
  echo "nmc3d_c1_load_config=$RUN_REL/config/C1_nmc3d_calib_load.yaml"
  echo "nmc3d_c4_load_config=$RUN_REL/config/C4_nmc3d_calib_load.yaml"
} >> "$RUN_DIR/manifest.txt"

echo "Running NMC3D calibration from run folder: $RUN_REL"

ROSCORE_PID=""

cleanup() {
  local status=$?
  if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "Stopping NMC3D calibration roscore..."
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roscore > "$RUN_DIR/roscore_nmc3d_calib.log" 2>&1 &
ROSCORE_PID=$!
sleep 3

"$CALIB_BIN" \
  "$VOCAB_PATH" \
  "$RUN_DIR/config/calib_finnforest_nmc3d.yaml" \
  "$RUN_DIR/config/C1_nmc3d_calib_load.yaml" \
  "$RUN_DIR/config/C4_nmc3d_calib_load.yaml" \
  2>&1 | tee "$RUN_DIR/nmc3d_calib.log"

if [[ -n "${ROSCORE_PID}" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -INT "$ROSCORE_PID" 2>/dev/null || true
  wait "$ROSCORE_PID" || true
  ROSCORE_PID=""
fi

{
  echo "nmc3d_calibration_finished_at=$(date -Iseconds)"
  echo "nmc3d_calibration_log=$RUN_REL/nmc3d_calib.log"
  echo "nmc3d_roscore_log=$RUN_REL/roscore_nmc3d_calib.log"
} >> "$RUN_DIR/manifest.txt"

echo "NMC3D calibration complete. Log saved to:"
echo "$RUN_DIR/nmc3d_calib.log"
