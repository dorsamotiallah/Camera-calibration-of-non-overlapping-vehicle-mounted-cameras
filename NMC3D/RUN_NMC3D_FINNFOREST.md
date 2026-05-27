# Run NMC3D On FinnForest C1/C4 Atlases

This runs only the NMC3D calibration stage using saved FinnForest C1/C4
ORB-SLAM atlases. It does not replay the rosbag.

## Inputs

For the controlled workflow, expected atlas files are inside a run-specific
folder:

```text
NMC3D/results_finnforest/<run_id>/c1_atlasCamera 1.osa
NMC3D/results_finnforest/<run_id>/c4_atlasCamera 2.osa
```

The orbcalib controlled calibration helper copies them there automatically when
the NMC3D folder is mounted into the `orbcalib` container:

```bash
docker exec -it orbcalib bash -lc 'cd /ws/src/orbcalib-master && tools/run_finnforest_controlled_calib.sh --run-id 40Hz_Controlled_Slam'
```

The older manual workflow expected atlas files in the `NMC3D` repo root:

```text
c1_atlasCamera 1.osa
c4_atlasCamera 2.osa
```

If you need to refresh them from `orbcalib-master`:

```bash
cp "/home/civit/Desktop/Dorsa/orbcalib-master/c1_atlasCamera 1.osa" \
   "/home/civit/Desktop/Dorsa/NMC3D/c1_atlasCamera 1.osa"

cp "/home/civit/Desktop/Dorsa/orbcalib-master/c4_atlasCamera 2.osa" \
   "/home/civit/Desktop/Dorsa/NMC3D/c4_atlasCamera 2.osa"
```

Check that the files are real atlases, not tiny placeholder files:

```bash
ls -lh "/home/civit/Desktop/Dorsa/NMC3D/c1_atlasCamera 1.osa"
ls -lh "/home/civit/Desktop/Dorsa/NMC3D/c4_atlasCamera 2.osa"
```

The current FinnForest atlases should be hundreds of MB.

## Avoid Docker Permission Problems

The Docker run below uses your host UID/GID:

```bash
--user "$(id -u):$(id -g)"
```

That makes new files in the mounted `NMC3D` folder owned by your normal host
user instead of `nobody:nogroup`.


## Configs

Use these FinnForest-specific configs:

```text
config/sim/calib_finnforest.yaml
config/sim/C1.yaml
config/sim/C4.yaml
```


## Controlled Run-Id Workflow

From the host, use the same `--run-id` that you used for the controlled
orbcalib run:

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
    tools/run_finnforest_nmc3d_calib.sh --run-id 13Hz_Controlled
  '
```

The helper:

1. verifies that the run folder contains the C1/C4 atlases,
2. creates temporary load configs in `results_finnforest/<run_id>/config/`,
3. points those configs at the run-specific atlas files,
4. builds the NMC3D `calib` target,
5. starts a temporary `roscore`,
6. runs NMC3D calibration,
7. saves logs in the same run folder.

Expected outputs:

```text
results_finnforest/<run_id>/nmc3d_calib.log
results_finnforest/<run_id>/roscore_nmc3d_calib.log
results_finnforest/<run_id>/config/C1_nmc3d_calib_load.yaml
results_finnforest/<run_id>/config/C4_nmc3d_calib_load.yaml
```

## Ground-Plane Scale Experiment

The ground-scale post-processing tool leaves the current NMC3D calibration
unchanged. It loads one saved monocular atlas, exports its keyframe/map-point
observations, runs scored RANSAC plane fitting, and prints the metric scale to
apply later to NMC3D/orbcalib translation results.

Build the atlas exporter in the same Docker environment used for NMC3D:

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
    cmake -S . -B build_nmc_docker
    cmake --build build_nmc_docker --target atlas_ground_export -j$(nproc)
  '
```

Then estimate scale for an atlas inside the same container. Replace the camera
height with the measured optical-center height above the ground:

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
    python3 tools/estimate_ground_scale.py \
      --atlas "c1_atlasCamera 1.osa" \
      --camera-height 1.35
  '
```

For run-specific atlases:

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" \
  --network host \
  --ipc=host \
  -e HOME=/tmp \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  orbcalib-noetic bash -lc '
    cd /ws/src/NMC3D
    python3 tools/estimate_ground_scale.py \
      --atlas "results_finnforest/<run_id>/c1_atlasCamera 1.osa" \
      --camera-height 1.35
  '
```

The important output line is:

```text
metric scale to apply to translations: <scale> m / SLAM unit
```

If the NMC3D/orbcalib translation is expressed in that atlas/map's SLAM units,
multiply the translation by this scale. Rotation is unchanged.

## Older One-Shot Docker Run

From the host:

```bash
cd /home/civit/Desktop/Dorsa/NMC3D
mkdir -p results_finnforest

docker run --rm -it \
  --user "$(id -u):$(id -g)" \
  --network host \
  --ipc=host \
  -e HOME=/tmp \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  orbcalib-noetic bash -lc '
    source /opt/ros/noetic/setup.bash
    cd /ws/src/NMC3D
    cmake -S . -B build_nmc_docker
    cmake --build build_nmc_docker --target calib -j$(nproc)
    roscore >/tmp/roscore.log 2>&1 &
    sleep 3
    mkdir -p results_finnforest
    ./build_nmc_docker/calib/calib \
      ./Vocabulary/ORBvoc.txt \
      config/sim/calib_finnforest.yaml \
      config/sim/C1.yaml \
      config/sim/C4.yaml \
      2>&1 | tee results_finnforest/nmc3d_finnforest_c1_c4.log
  '
```

After the run, the result log should be writable from the host:

```bash
ls -lh /home/civit/Desktop/Dorsa/NMC3D/results_finnforest
```

## What To Look For

The run should load:

```text
./c1_atlasCamera 1.osa
./c4_atlasCamera 2.osa
```

NMC3D-specific output includes lines such as:

```text
frame-to-frame distance-bin selection
global final depth selection
matched mps selected during frame-to-frame matching size
```

The final result, if calibration succeeds, is printed as:

```text
---- first pose optim ----
euler: ...
trans: ...
---- final pose optim ----
euler: ...
trans: ...
```

If it prints `no common features detected!!`, then NMC3D also failed to accept
geometrically valid C1/C4 common map regions from these atlases.
