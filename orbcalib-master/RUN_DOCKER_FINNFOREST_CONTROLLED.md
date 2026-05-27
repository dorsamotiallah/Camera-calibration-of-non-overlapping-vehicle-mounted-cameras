# Runbook: FinnForest Controlled SLAM Playback

This runbook adds a reproducible, ACK-driven SLAM atlas-building path for
FinnForest C1/C4.

It does **not** replace [RUN_DOCKER_FINNFOREST.md](/home/civit/Desktop/Dorsa/orbcalib-master/RUN_DOCKER_FINNFOREST.md).
The old `rosbag play` workflow still works.

## What This Adds

Normal playback:

```text
rosbag play -> /cam_c1/image, /cam_c4/image -> orbcalib / ORB-SLAM
```

Controlled playback:

```text
controlled player -> /cam_c1/image, /cam_c4/image -> orbcalib / ORB-SLAM
                  <- /orbcalib/camera1/processed
                  <- /orbcalib/camera2/processed
```

The controlled player publishes normal ROS image topics, then waits for ACKs
that orbcalib publishes after `TrackMonocular(...)` returns. This prevents the
ROS image queue from growing ahead of ORB-SLAM.

## Files Added

```text
config/sim/calib_finnforest_controlled.yaml
tools/controlled_finnforest_bag_player.py
tools/run_finnforest_controlled_slam.sh
tools/run_finnforest_controlled_calib.sh
```

The normal config also contains disabled ACK settings:

```yaml
PlaybackAck.Enabled: 0
PlaybackAck.TopicPrefix: "/orbcalib"
PlaybackAck.LogEvery: 100
```

## One-Time Rebuild

The ACK implementation uses `std_msgs/Header`, so rebuild the calibration
binary once inside the Docker container:

```bash
sudo chown -R "$(id -u):$(id -g)" ~/Desktop/Dorsa/orbcalib-master/build 2>/dev/null || true
docker exec -it orbcalib bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/orbcalib-master && cmake --build build --target calib -j$(nproc)'
```

If your build directory is different, rebuild the same way you normally build
`./build/calib/calib`.

## Start Docker Container

Use the same container startup from `RUN_DOCKER_FINNFOREST.md`. The important
part is to run the container as your host user so generated files stay writable:

```bash
cd ~/Desktop/Dorsa/orbcalib-master

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
  --volume "$HOME/Desktop/Dorsa/NMC3D":/ws/src/NMC3D \
  --device /dev/dri:/dev/dri \
  orbcalib-noetic bash
```

Keep this shell open.

## Run Controlled SLAM Phase

From a host terminal:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_slam.sh'
```

This script does the full SLAM atlas-building phase:

1. creates a timestamped result folder,
2. generates temporary C1/C4 SLAM configs that save atlases into that folder,
3. starts `roscore`,
4. starts `./build/calib/calib` in `slam` mode with ACKs enabled,
5. runs the controlled bag player,
6. waits until the final C1/C4 frames are processed,
7. sends `SIGINT` to orbcalib, equivalent to pressing `Ctrl+C`,
8. waits for ORB-SLAM shutdown and atlas saving,
9. writes logs and a manifest.

## Output Folder

Each run writes to:

```text
results_finnforest/<timestamp>_c1_c4_controlled/
```

Expected contents:

```text
config/
manifest.txt
roscore.log
slam.log
player.log
c1_atlasCamera 1.osa
c4_atlasCamera 2.osa
```

The atlases are saved directly inside the run folder, so they do not overwrite
old atlases in the repo root.

## To see the live slam logs run this command in another terminal:

```
tail -f /home/civit/Desktop/Dorsa/orbcalib-master/results_finnforest/40Hz_Controlled_Slam/slam.log
```

and this player log in another termina: 

```
tail -f /home/civit/Desktop/Dorsa/orbcalib-master/results_finnforest/40Hz_Controlled_Slam/player.log
```

## Useful Options

Use a custom run id:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_slam.sh --run-id test_c1_c4_controlled'
```

Allow a small amount of buffered work:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_slam.sh --max-in-flight 3'
```

Use a different bag:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_slam.sh --bag /ws/src/orbcalib-master/data/S01_C1_C4_40Hz.bag'
```

Recommended first run:

```text
--max-in-flight 1
```

This is the strictest setting: publish one C1/C4 image pair, wait for both ACKs,
then publish the next pair.

## After SLAM: Run Calibration

To calibrate from a controlled run, pass the same run id to the calibration
helper:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_calib.sh --run-id 13Hz_Controlled'
```

The helper:

1. verifies the run folder contains C1/C4 atlases,
2. copies the same atlases to `NMC3D/results_finnforest/<run_id>/` when
   `/ws/src/NMC3D` is mounted,
3. creates temporary load configs inside the run folder,
4. points those configs at that run's atlas prefixes,
5. runs orbcalib in `calib` mode,
6. saves the calibration output as `calib.log` in the same run folder,
7. appends calibration metadata to `manifest.txt`.

If your existing `orbcalib` container was started before the NMC3D volume was
added, atlas export will be skipped with a clear message. Restart the container
with the `--volume "$HOME/Desktop/Dorsa/NMC3D":/ws/src/NMC3D` line above, or
pass the path explicitly if it is mounted somewhere else:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_calib.sh --run-id 40Hz_Controlled_Slam --nmc3d-dir /ws/src/NMC3D'
```

Example generated atlas load path:

```yaml
System.LoadAtlasFromFile: "results_finnforest/40Hz_Controlled_Slam/c1_atlas"
```

and:

```yaml
System.LoadAtlasFromFile: "results_finnforest/40Hz_Controlled_Slam/c4_atlas"
```

The useful calibration result is usually near the end of:

```text
results_finnforest/<run_id>/calib.log
```

Look for:

```text
---- first pose optim ----
euler: ...
trans: ...
```

If `final pose optim` reports `0 vertices to optimize`, use `first pose optim`
as the meaningful result, as in the earlier FinnForest runs.

## Run NMC3D Calibration From The Same Run

After the orbcalib calibration helper has exported the atlases, NMC3D should
contain:

```text
/home/civit/Desktop/Dorsa/NMC3D/results_finnforest/<run_id>/
  c1_atlasCamera 1.osa
  c4_atlasCamera 2.osa
```

Run the NMC3D calibration automation from the host:

```bash
cd /home/civit/Desktop/Dorsa/NMC3D

docker run --rm -it \
  --user "$(id -u):$(id -g)" \
  --network host \
  --ipc=host \
  -e HOME=/tmp \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  orbcalib-noetic bash -lc '
    cd /ws/src/NMC3D
    tools/run_finnforest_nmc3d_calib.sh --run-id 40Hz_Controlled_Slam
  '
```

This writes:

```text
NMC3D/results_finnforest/<run_id>/nmc3d_calib.log
NMC3D/results_finnforest/<run_id>/roscore_nmc3d_calib.log
NMC3D/results_finnforest/<run_id>/config/
```

## Notes

- The controlled player still publishes ROS image topics.
- The ACK topics are only feedback for pacing; ORB-SLAM input is unchanged.
- The old `rosbag play` workflow is still available.
- If ACKs stop arriving, the player fails with a timeout instead of silently
  flooding queues or dropping frames.
