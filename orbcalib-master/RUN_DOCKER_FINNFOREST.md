# Runbook: FinnForest + Docker + ROS 1 Replay

This file documents a separate FinnForest workflow for `orbcalib-master`.
It does not replace or modify [RUN_DOCKER.md](/home/civit/Desktop/Dorsa/orbcalib-master/RUN_DOCKER.md). Keep using `RUN_DOCKER.md` for the existing Gazebo / MCAP setup.

This FinnForest workflow uses:

1. the existing `orbcalib-noetic` Docker image
2. a ROS 1 container for `roscore` and the calibration executable
3. the ROS 1 bag already generated at:

```text
~/Desktop/Dorsa/orbcalib-master/data/S01_C1_C4_40Hz.bag
```

Use the generated bag with `rosbag play`. This runbook intentionally does not
use the online FinnForest PNG publisher.

For ACK-driven playback that waits for ORB-SLAM to finish processing frames
before publishing too far ahead, use
[RUN_DOCKER_FINNFOREST_CONTROLLED.md](/home/civit/Desktop/Dorsa/orbcalib-master/RUN_DOCKER_FINNFOREST_CONTROLLED.md).

## Assumptions

- Host OS: Ubuntu
- Repo path on host:
  - `~/Desktop/Dorsa/orbcalib-master`
- FinnForest ROS1 bag on host:
  - `~/Desktop/Dorsa/orbcalib-master/data/S01_C1_C4_40Hz.bag`
- Docker image name:
  - `orbcalib-noetic`
- Main ROS 1 container name:
  - `orbcalib`

## One-Time Setup

### 1. Build the Docker image

Run from the repo root:

```bash
cd ~/Desktop/Dorsa/orbcalib-master
docker build --progress=plain -t orbcalib-noetic -f docker/noetic.Dockerfile .
```

### 2. Allow GUI access for Docker

Run once per login session:

```bash
xhost +local:docker
```

### 3. Repair old Docker-owned outputs, if needed

If a previous run created atlases or result folders owned by `nobody:nogroup`,
repair the ownership once from the host:

```bash
cd ~/Desktop/Dorsa/orbcalib-master
sudo chown -R "$(id -u):$(id -g)" results_finnforest 2>/dev/null || true
sudo chown "$(id -u):$(id -g)" ./*atlasCamera*.osa 2>/dev/null || true
```

This is only needed for files that already exist with bad permissions. The
Docker commands below run as your host user, so new files should be writable
from the host.

## Start the Main Container

Run these commands from the repo root when you want a fresh FinnForest run:

```bash
xhost +local:docker

docker rm -f orbcalib 2>/dev/null || true

docker run --rm -it \
  --name orbcalib \
  --user "$(id -u):$(id -g)" \
  --network host \
  --ipc=host \
  --env DISPLAY=$DISPLAY \
  --env HOME=/tmp \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$HOME/Desktop/Dorsa/orbcalib-master":/ws/src/orbcalib-master \
  --device /dev/dri:/dev/dri \
  orbcalib-noetic bash
```

Important:
- Keep this first shell open.
- The container remains alive while this shell is open.
- Use new terminals with `docker exec` for the remaining commands.
- The FinnForest bag is available through the repo mount at:
  - `/ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag`

## Quick Sanity Checks

Run these before starting a calibration session:

```bash
docker exec -it orbcalib bash -lc 'ls -lh /ws/src/orbcalib-master/Vocabulary/ORBvoc.txt'
docker exec -it orbcalib bash -lc 'ls -lh /ws/src/orbcalib-master/build/calib/calib'
docker exec -it orbcalib bash -lc 'ls -lh /ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag'
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rosbag info /ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag'
```

Expected bag topics:

- `/cam_c1/image`
- `/cam_c4/image`

## FinnForest Camera Topics Used

This workflow uses:

- `/cam_c1/image`
- `/cam_c4/image`

## FinnForest Camera Configs

The repo already contains FinnForest monocular camera YAML files:

- `config/sim/C1.yaml`
- `config/sim/C4.yaml`

They correspond to:

- C1 = serial `22573022`
- C4 = serial `22555668`

These camera files assume:

```yaml
Camera.width: 1920
Camera.height: 1200
Camera.fps: 40
Camera.RGB: 1
```

The existing bag already uses these topics and RGB image encoding, matching
`Camera.RGB: 1`.

## Phase A: Build Atlases (`slam` mode)

### 1. Use the separate FinnForest calib config

This workflow uses:

```text
config/sim/calib_finnforest.yaml
```

It already contains the monocular FinnForest topic names:

```yaml
Mode: slam
UseViewer: 0
Camera1.Type: 0
Camera1.Image: "/cam_c1/image"

Camera2.Type: 0
Camera2.Image: "/cam_c4/image"
```

No depth topics are used in monocular mode.

`UseViewer: 0` disables only the Pangolin/OpenCV viewer windows. It does not
change tracking, atlas saving, or calibration logic.

### 2. Set camera atlas behavior

Edit `config/sim/C1.yaml`:

```yaml
System.SaveAtlasToFile: "c1_atlas"
# System.LoadAtlasFromFile: "c1_atlas"
```

Edit `config/sim/C4.yaml`:

```yaml
System.SaveAtlasToFile: "c4_atlas"
# System.LoadAtlasFromFile: "c4_atlas"
```

### 3. Start ROS master

Open Terminal 1:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && roscore'
```

Leave it running.

### 4. Start the calibration executable in `slam` mode

Before each new FinnForest slam run, delete old atlases so you do not mix runs:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && rm -f ./*atlasCamera*.osa'
```

Open Terminal 2:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && export LIBGL_ALWAYS_SOFTWARE=1 && ./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib_finnforest.yaml config/sim/C1.yaml config/sim/C4.yaml'
```

Leave it running.

### 5. Replay the existing FinnForest ROS 1 bag

Open Terminal 3:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rosbag play --wait-for-subscribers -r 0.25 --clock /ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag'

```

Leave it running until playback finishes. The bag is about `360G`, so the
full 40 Hz replay can take a while and can generate large atlases.

If tracking is unstable or the process exits before saving atlases, replay more
slowly:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rosbag play -r 0.25 --clock /ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag'
```

### 6. Optional topic verification

Open Terminal 4:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic list'
```

Expected topics:

- `/cam_c1/image`
- `/cam_c4/image`

Optional rate checks:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic hz /cam_c1/image'
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic hz /cam_c4/image'
```

### 7. Stop `slam` cleanly after replay ends

When Terminal 3 exits after `rosbag play` finishes, go to Terminal 2 and press
`Ctrl+C`.

This clean shutdown is what saves the atlas files.

### 8. Confirm atlas files exist

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && ls -lah | grep atlas'
```

Expected files:

- `c1_atlasCamera 1.osa`
- `c4_atlasCamera 2.osa`

They should be owned by your host user, not `nobody:nogroup`. If they are still
owned by `nobody:nogroup`, stop and run the ownership repair command in
`One-Time Setup` before copying or deleting them from the host.

If either atlas is missing or only a few bytes, replay did not feed enough
trackable frames into that camera. Run the slam phase again and watch Terminal 2
for ORB-SLAM tracking state messages.

### 9. Save a copy of the atlases

From the host, copy the finished atlases into `results_finnforest`:

```bash
cd ~/Desktop/Dorsa/orbcalib-master
mkdir -p results_finnforest
cp -v "c1_atlasCamera 1.osa" "results_finnforest/c1_atlasCamera 1.osa"
cp -v "c4_atlasCamera 2.osa" "results_finnforest/c4_atlasCamera 2.osa"
ls -lh results_finnforest/*atlasCamera*.osa
```

If this fails with `Permission denied`, run the ownership repair command in
`One-Time Setup`, then repeat the copy.

## Phase B: Run Calibration (`calib` mode)

### 1. Switch `calib_finnforest.yaml` to calibration mode

Edit `config/sim/calib_finnforest.yaml`:

```yaml
Mode: calib

Camera1.Type: 0
Camera1.Image: "/cam_c1/image"

Camera2.Type: 0
Camera2.Image: "/cam_c4/image"
```

### 2. Set camera configs to load atlases

Edit `config/sim/C1.yaml`:

```yaml
# System.SaveAtlasToFile: "c1_atlas"
System.LoadAtlasFromFile: "c1_atlas"
```

Edit `config/sim/C4.yaml`:

```yaml
# System.SaveAtlasToFile: "c4_atlas"
System.LoadAtlasFromFile: "c4_atlas"
```

### 3. Run calibration without replay

Use a new terminal:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && mkdir -p results_finnforest && ./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib_finnforest.yaml config/sim/C1.yaml config/sim/C4.yaml 2>&1 | tee results_finnforest/finnforest_c1_c4_calib.log'
```

No FinnForest replay is needed in this phase.

The important output lines are printed near the end:

```text
---- first pose optim ----
euler: ...
trans: ...
---- final pose optim ----
euler: ...
trans: ...
```

The same output is saved in:

```text
results_finnforest/finnforest_c1_c4_calib.log
```

Because the main container was started with `--user "$(id -u):$(id -g)"`, this
log file should also be writable from the host.

## Compare With FinnForest Ground Truth

The repository contains a helper in the FinnForest folder that chains the
official C1-C2, C2-C3, and C3-C4 calibration files into C1-C4.

Run it from the host:

```bash
python3 ~/Desktop/Dorsa/Finnforest/Groundtruth_T14/calc_finnforest_t14.py
```

Useful reference values printed by that helper:

```text
T14_yaml_cammap_style
euler: -2.301785874 55.859820876 -2.481486448
trans_m: -0.591602234 -0.003443787 0.083303229

T41_yaml_cammap_style
euler: 0.441285582 -55.890011087 1.027003149
trans_m: 0.400571039 0.011266766 0.443125708

T14_opencv_style_cammap_style
euler: -0.602037826 -55.909690436 0.836021159
trans_m: -0.581736657 0.006461918 -0.296923530

T41_opencv_style_cammap_style
euler: -0.161168055 55.911677697 -0.602053546
trans_m: 0.571873971 -0.011636352 -0.315353191
```

Use both directions (`T14` and `T41`) when comparing because orbcalib prints
the map-to-map transform it estimates from the two saved atlases, and the
direction can be opposite of the convention used in the FinnForest files.

## Notes

- This workflow does not change or replace the existing Gazebo / MCAP setup.
- Keep using `RUN_DOCKER.md` for the old workflow.
- Use `RUN_DOCKER_FINNFOREST.md` only for FinnForest C1/C4 runs.
- The current camera configs are set for slam mode by default. After you
  switch `C1.yaml`, `C4.yaml`, and `calib_finnforest.yaml` to calibration mode,
  switch them back before another atlas-building run.
