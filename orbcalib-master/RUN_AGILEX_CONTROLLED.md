# Runbook: Agilex Controlled SLAM And Calibration

This runbook uses the existing automation scripts for Agilex PNG folders:

- `tools/run_agilex_controlled_slam.sh`
- `tools/run_agilex_controlled_calib.sh`
- `NMC3D/tools/run_finnforest_nmc3d_calib.sh` with Agilex parameter overrides

The controlled SLAM script starts `roscore`, runs orbcalib/ORB-SLAM in `slam`
mode, publishes PNG frame pairs with ACK backpressure, stops ORB-SLAM with
`SIGINT`, and saves atlases plus logs into a result folder.

## 1. Start The Docker Container

From the host:

```bash
cd ~/Desktop/Dorsa/orbcalib-master

xhost +local:docker

docker rm -f orbcalib 2>/dev/null || true

docker run --rm -it \
  --name orbcalib \
  --user "$(id -u):$(id -g)" \
  --network host \
  --ipc=host \
  --env DISPLAY="$DISPLAY" \
  --env HOME=/tmp \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$HOME/Desktop/Dorsa/orbcalib-master":/ws/src/orbcalib-master \
  --volume "$HOME/Desktop/Dorsa/NMC3D":/ws/src/NMC3D \
  --volume "$HOME/Desktop/Dorsa/Agilex Recordings":/ws/src/Agilex_Recordings \
  --volume "/media/civit/T7":/ws/src/T7 \
  --device /dev/dri:/dev/dri \
  orbcalib-noetic bash
```

Keep that shell open. Run the commands below from another host terminal with
`docker exec`.

## 2. One-Time Build Check

Run this after code changes, or if `build/calib/calib` is missing:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && cmake --build build --target calib -j$(nproc)'
```

Quick sanity check:

```bash
docker exec -it orbcalib bash -lc 'ls -lh /ws/src/orbcalib-master/Vocabulary/ORBvoc.txt /ws/src/orbcalib-master/build/calib/calib'
docker exec -it orbcalib bash -lc 'find /ws/src/Agilex_Recordings/Agilex\ recordings\ 27.5.2026 -maxdepth 2 -type d | sort'
```

## 3. Controlled SLAM: Back To Front

Recommended first run on `bigLoopNoTilt`. This order matches ground-truth
`T_front_back`, because orbcalib estimates camera 1 to camera 2:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/bigLoopNoTilt" \
  --camera1 back \
  --camera2 front \
  --run-id agilex_bigLoopNoTilt_back_front \
  --pairing nearest \
  --max-skew-sec 0.05 \
  --max-in-flight 1 \
  --playback-rate 0'
```

To turn on the ORB-SLAM viewer with the same settings, add `--viewer`:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/bigLoopNoTilt" \
  --camera1 back \
  --camera2 front \
  --run-id agilex_bigLoopNoTilt_back_front \
  --max-in-flight 1 \
  --playback-rate 0 \
  --viewer'
```

This writes:

```text
results_agilex/agilex_bigLoopNoTilt_back_front/
  config/
  manifest.txt
  roscore.log
  slam.log
  player.log
  back_atlasCamera 1.osa
  front_atlasCamera 2.osa
```

Watch progress from the host:

```bash
tail -f ~/Desktop/Dorsa/orbcalib-master/results_agilex/agilex_bigLoopNoTilt_back_front/slam.log
tail -f ~/Desktop/Dorsa/orbcalib-master/results_agilex/agilex_bigLoopNoTilt_back_front/player.log
```

For a quick smoke test, limit the number of frame pairs:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/bigLoopNoTilt" \
  --camera1 back \
  --camera2 front \
  --run-id smoke_back_front_300 \
  --max-pairs 300 \
  --max-in-flight 1 \
  --playback-rate 0'
```

## 4. Other Agilex SLAM Runs

Same back/front pair, tilted-down big loop:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/bigLoopTiltedDown" \
  --camera1 back \
  --camera2 front \
  --run-id agilex_bigLoopTiltedDown_back_front \
  --max-in-flight 1 \
  --playback-rate 0'
```

Same back/front pair, small tilted-down loop:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/smallLoopTiltedDown" \
  --camera1 back \
  --camera2 front \
  --run-id agilex_smallLoopTiltedDown_back_front \
  --max-in-flight 1 \
  --playback-rate 0'
```

Note: `calib_agilex_controlled.yaml` now subscribes camera 1 to
`/cam_back/image` and camera 2 to `/cam_front/image`. The normal recommended
ordering for ground-truth comparison is `--camera1 back --camera2 front`.

## 5. orbcalib Calibration From Saved Atlases

Run orbcalib calibration on the saved back/front atlases:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_calib.sh \
  --run-id agilex_bigLoopNoTilt_back_front \
  --camera1 back \
  --camera2 front'
```

To turn on the viewer during calibration loading too, add `--viewer`:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_calib.sh \
  --run-id agilex_bigLoopNoTilt_back_front \
  --camera1 back \
  --camera2 front \
  --viewer'
```

This appends to the same result folder:

```text
roscore_calib.log
calib.log
config/back_controlled_calib_load.yaml
config/front_controlled_calib_load.yaml
config/calib_agilex_calib.yaml
```

To also copy the atlases into `NMC3D/results_agilex/<run_id>/`:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_calib.sh \
  --run-id agilex_bigLoopNoTilt_back_front \
  --camera1 back \
  --camera2 front \
  --export-nmc3d \
  --nmc3d-dir /ws/src/NMC3D'
```

The useful orbcalib result is near the end of:

```text
results_agilex/agilex_bigLoopNoTilt_back_front/calib.log
```

## 6. NMC3D Calibration From The Same Agilex Atlases

The current NMC3D helper is named for FinnForest and expects atlas aliases named
`c1_atlas` and `c4_atlas`. Until there is a dedicated Agilex NMC3D helper, use
the existing script with Agilex config overrides and symlink aliases.

First export the orbcalib atlases to NMC3D:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_agilex_controlled_calib.sh \
  --run-id agilex_bigLoopNoTilt_back_front \
  --camera1 back \
  --camera2 front \
  --export-nmc3d \
  --nmc3d-dir /ws/src/NMC3D'
```

Then run NMC3D calibration:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && \
  cd /ws/src/NMC3D && \
  RUN_DIR=/ws/src/NMC3D/results_agilex/agilex_bigLoopNoTilt_back_front && \
  ln -sf "back_atlasCamera 1.osa" "$RUN_DIR/c1_atlasCamera 1.osa" && \
  ln -sf "front_atlasCamera 2.osa" "$RUN_DIR/c4_atlasCamera 2.osa" && \
  CALIB_CONFIG=/ws/src/orbcalib-master/config/sim/calib_agilex.yaml \
  C1_CONFIG=/ws/src/orbcalib-master/config/sim/agilex_back_cam.yaml \
  C4_CONFIG=/ws/src/orbcalib-master/config/sim/agilex_front_cam.yaml \
  tools/run_finnforest_nmc3d_calib.sh \
    --run-dir "$RUN_DIR"'
```

To skip rebuilding NMC3D after it has already been built:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && \
  cd /ws/src/NMC3D && \
  RUN_DIR=/ws/src/NMC3D/results_agilex/agilex_bigLoopNoTilt_back_front && \
  CALIB_CONFIG=/ws/src/orbcalib-master/config/sim/calib_agilex.yaml \
  C1_CONFIG=/ws/src/orbcalib-master/config/sim/agilex_back_cam.yaml \
  C4_CONFIG=/ws/src/orbcalib-master/config/sim/agilex_front_cam.yaml \
  tools/run_finnforest_nmc3d_calib.sh \
    --run-dir "$RUN_DIR" \
    --skip-build'
```

The NMC3D output is saved to:

```text
NMC3D/results_agilex/agilex_bigLoopNoTilt_back_front/nmc3d_calib.log
```

## 7. Reusing The Same Pattern

Use these parameters as the knobs you change between runs:

```text
--dataset-root "/ws/src/Agilex_Recordings/Agilex recordings 27.5.2026/<loop_name>"
--camera1 back
--camera2 front
--run-id <descriptive_result_name>
--max-pairs <N>
--start-index <N>
--hz <rate>
--max-in-flight 1
--viewer
```

The safest default is:

```text
--max-in-flight 1 --playback-rate 0
```

That publishes one synchronized frame pair, waits until both ORB-SLAM tracking
calls return, then publishes the next pair.

## 8. Batch Defish Saved Frames

To defish all PNG/JPG frames under a dataset root with `front`, `back`, `left`,
and `right` subfolders:

```bash
cd ~/Desktop/Dorsa

python3 orbcalib-master/tools/batch_defish_agilex_frames.py \
  "Agilex Recordings/Agilext 8.6.2026/tinyLoop" \
  --fov-deg 120
```

By default this reads the per-camera intrinsics from
`orbcalib-master/config/sim/agilex_<camera>_cam.yaml` and writes beside the
input dataset as:

```text
Agilex Recordings/Agilext 8.6.2026/tinyLoop_defished/
  front/
  back/
  left/
  right/
```

Useful options:

```text
--fov-deg 100                         preserve a narrower output view
--fov-axis horizontal|vertical|diagonal
--output-root /path/to/output
--output-size 1920x1080
--cameras front back
--calib front=/path/to/front.yaml     override one camera calibration
--overwrite
--dry-run --limit 5
```
