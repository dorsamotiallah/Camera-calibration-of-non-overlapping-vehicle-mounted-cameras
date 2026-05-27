# Runbook: Docker + ROS 1 + MCAP Replay

This file documents the full workflow for running this repository unchanged inside Docker, using a ROS 2 `.mcap` recording as input and replaying it into ROS 1 image topics.

The workflow has two phases:

1. `slam` phase
   Build and save atlases for the selected cameras.
2. `calib` phase
   Load the saved atlases and run camera-to-camera calibration.

## Assumptions

- Host OS: Ubuntu
- Repo path on host:
  - `~/Desktop/Dorsa/orbcalib-master`
- Docker image name:
  - `orbcalib-noetic`
- Docker container name:
  - `orbcalib`
- MCAP file path inside repo:
  - `data/warehouse_rgbd_run_01_0.mcap`

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

## Start the Container

Run these commands from the repo root every time you want a fresh run:

```bash
xhost +local:docker

docker rm -f orbcalib 2>/dev/null || true

docker run --rm -it \
  --name orbcalib \
  --network host \
  --ipc=host \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$PWD":/ws/src/orbcalib-master \
  --device /dev/dri:/dev/dri \
  orbcalib-noetic bash
```

Important:
- Keep this first shell open.
- The container remains alive while this shell is open.
- Use new terminals with `docker exec` for the remaining commands.

Verify the container is running:

```bash
docker ps
```

## Sanity Checks

Run these before starting a new session:

```bash
docker exec -it orbcalib bash -lc 'ls -lah /ws/src/orbcalib-master/Vocabulary/ORBvoc.txt'
docker exec -it orbcalib bash -lc 'ls -lah /ws/src/orbcalib-master/data/house_rgbd_run_03_0.mcap'
docker exec -it orbcalib bash -lc 'ls -lah /ws/src/orbcalib-master/build/calib/calib'
```

All three files must exist.

## Camera Topics Used

This workflow uses:

- `/cam_front/image`
- `/cam_front/depth_image`
- `/cam_back/image`
- `/cam_back/depth_image`

## Camera Config Values

The current Gazebo camera setup corresponds to these values in both camera YAML files:

```yaml
Camera.width: 640
Camera.height: 480
Camera.fps: 10
Camera1.fx: 319.9987983703613
Camera1.fy: 319.998779296875
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera1.k3: 0.0
RGBD.DepthMapFactor: 1.0
```

## Phase A: Build Atlases (`slam` mode)

### 1. Set YAML files for slam mode

Edit `config/sim/calib.yaml`:

```yaml
Mode: slam
```

Edit `config/sim/front_cam.yaml`:

```yaml
System.SaveAtlasToFile: "front_atlas"
# System.LoadAtlasFromFile: "front_atlas"
```

Edit `config/sim/back_cam.yaml`:

```yaml
System.SaveAtlasToFile: "back_atlas"
# System.LoadAtlasFromFile: "back_atlas"
```

### 2. Start ROS master

Open Terminal 1:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && roscore'
```

Leave it running.

### 3. Start the calibration executable in slam mode

Before each new slam run, delete old atlases so you do not mix runs:

``` bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && rm -f ./*atlasCamera*.osa'
```

Open Terminal 2:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && export LIBGL_ALWAYS_SOFTWARE=1 && ./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib.yaml config/sim/front_cam.yaml config/sim/back_cam.yaml'
```

Leave it running.

### 4. Replay the MCAP file into ROS 1 topics

Open Terminal 3:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && python3 tools/play_mcap_to_ros1.py /ws/src/orbcalib-master/data/house_rgbd_broader_run_02_0.mcap'
```

Leave it running until playback finishes.

### 5. Optional topic verification

Open Terminal 4:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic list'
```

Expected topics:

- `/cam_front/image`
- `/cam_front/depth_image`
- `/cam_back/image`
- `/cam_back/depth_image`

Optional rate checks:

```bash
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic hz /cam_front/image'
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && rostopic hz /cam_back/image'
```

### 6. Stop slam cleanly after replay ends

When Terminal 3 finishes replaying:

- go to Terminal 2
- press `Ctrl+C`

This clean shutdown is what saves the atlas files.

### 7. Confirm atlas files exist

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && ls -lah | grep atlas'
```

Expected files:

- `front_atlasCamera 1.osa`
- `back_atlasCamera 2.osa`

## Phase B: Run Calibration (`calib` mode)

### 1. Set YAML files for calibration mode

Edit `config/sim/calib.yaml`:

```yaml
Mode: calib
```

Edit `config/sim/front_cam.yaml`:

```yaml
# System.SaveAtlasToFile: "front_atlas"
System.LoadAtlasFromFile: "front_atlas"
```

Edit `config/sim/back_cam.yaml`:

```yaml
# System.SaveAtlasToFile: "back_atlas"
System.LoadAtlasFromFile: "back_atlas"
```

### 2. Run calibration without replay

Use a new terminal:

```bash

docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && ./build/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib.yaml config/sim/front_cam.yaml config/sim/back_cam.yaml'
```

No MCAP replay is needed in this phase.

## Interpreting Results

### Successful calibration

If the run succeeds, it should print the estimated camera-to-camera transform in the terminal.

### Failed calibration

If the run ends with messages like:

```text
optim sim3 get size: 0
no common features detected!!
```

then:

- the code executed correctly
- the saved maps loaded correctly
- but the algorithm could not find a stable final calibration from the available map overlap / matches

In that case the next things to revisit are:

- trajectory quality
- amount of scene texture
- revisits / loop closure quality
- front/back common map content

## Stop and Clean Up

To stop running commands:

- use `Ctrl+C` in each terminal

To remove the container after the session:

```bash
docker rm -f orbcalib
```

## Fast Re-Run Summary

Every later run follows this pattern:

1. `cd ~/Desktop/Dorsa/orbcalib-master`
2. `xhost +local:docker`
3. start container with `docker run ...`
4. set YAMLs to `slam`
5. run `roscore`
6. run `./build/calib/calib ...`
7. run `python3 tools/play_mcap_to_ros1.py ...`
8. wait for replay to finish
9. stop slam process with `Ctrl+C`
10. confirm atlas files exist
11. set YAMLs to `calib`
12. run `./build/calib/calib ...` again without replay
