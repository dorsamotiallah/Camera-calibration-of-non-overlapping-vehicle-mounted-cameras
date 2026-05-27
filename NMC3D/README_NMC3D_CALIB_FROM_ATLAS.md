# NMC3D Calibration From Saved Atlases

This folder is a separate experimental copy of `orbcalib-master`. Use it to test the
NMC3D-style depth-balanced feature selection without changing the original CamMap code.

The workflow below assumes SLAM has already been run and the atlas files already exist.
It only runs the calibration stage.

## 1. Build the Docker Image

From the `NMC3D` folder:

```bash
cd /home/civit/Desktop/Dorsa/NMC3D
docker build --progress=plain -t nmc3d-noetic -f docker/noetic.Dockerfile .
```

If you already built the original image as `orbcalib-noetic`, you can reuse that image
instead of building `nmc3d-noetic`. The code is mounted from the host at run time.

## 2. Build or Rebuild the NMC3D Calibration Binary

Run this after every source-code change before launching calibration. The code is mounted
from the host, but the executable in `build_nmc_docker/calib/calib` is not updated until
you rebuild it.

```bash
docker run --rm -it \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  nmc3d-noetic bash -lc '
    source /opt/ros/noetic/setup.bash &&
    cd /ws/src/NMC3D &&
    cmake -S . -B build_nmc_docker &&
    cmake --build build_nmc_docker --target calib -j$(nproc)
  '
```

If you are reusing the original image, replace `nmc3d-noetic` with `orbcalib-noetic`.

## 3. Put Saved Atlas Files In NMC3D

The camera YAML files currently load:

```yaml
System.LoadAtlasFromFile: "front_atlas"
System.LoadAtlasFromFile: "back_atlas"
```

ORB-SLAM expects these files in the repo root with the camera suffix:

```text
front_atlasCamera 1.osa
back_atlasCamera 2.osa
```

If the atlases were created in `orbcalib-master`, copy them into `NMC3D`:

```bash
cp "/home/civit/Desktop/Dorsa/orbcalib-master/front_atlasCamera 1.osa" \
   "/home/civit/Desktop/Dorsa/NMC3D/front_atlasCamera 1.osa"

cp "/home/civit/Desktop/Dorsa/orbcalib-master/back_atlasCamera 2.osa" \
   "/home/civit/Desktop/Dorsa/NMC3D/back_atlasCamera 2.osa"
```

Check that they exist:

```bash
ls -lh "/home/civit/Desktop/Dorsa/NMC3D/front_atlasCamera 1.osa"
ls -lh "/home/civit/Desktop/Dorsa/NMC3D/back_atlasCamera 2.osa"
```

## 4. Check Calibration Mode

Make sure `config/sim/calib.yaml` is in calibration mode:

```yaml
Mode: calib
```

Make sure the camera configs are loading, not saving, atlases:

```yaml
System.LoadAtlasFromFile: "front_atlas"
# System.SaveAtlasToFile: "front_atlas"
```

```yaml
System.LoadAtlasFromFile: "back_atlas"
# System.SaveAtlasToFile: "back_atlas"
```

## 5. Rebuild And Run Calibration

Allow GUI access once per login session:

```bash
xhost +local:docker
```

Even in `Mode: calib`, the executable calls `ros::start()`, so a ROS master must
be running. The safest repeatable command is this rebuild-and-run version. Use it after
each code change so you do not accidentally run a stale binary:

```bash
docker run --rm -it \
  --network host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  --device /dev/dri:/dev/dri \
  nmc3d-noetic bash -lc '
    source /opt/ros/noetic/setup.bash
    cd /ws/src/NMC3D
    cmake -S . -B build_nmc_docker
    cmake --build build_nmc_docker --target calib -j$(nproc)
    roscore >/tmp/roscore.log 2>&1 &
    sleep 3
    export LIBGL_ALWAYS_SOFTWARE=1
    ./build_nmc_docker/calib/calib \
      ./Vocabulary/ORBvoc.txt \
      config/sim/calib.yaml \
      config/sim/front_cam.yaml \
      config/sim/back_cam.yaml
  '
```

If you are reusing the original image, replace `nmc3d-noetic` with `orbcalib-noetic`.

Alternatively, use the same style as `orbcalib-master`: keep one container running, start
`roscore` in one terminal, then rebuild and run the calibration executable in another
terminal with `docker exec`.

Terminal 1, create and enter the container:

```bash
docker rm -f nmc3d 2>/dev/null || true
docker run -it \
  --name nmc3d \
  --network host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/civit/Desktop/Dorsa/NMC3D:/ws/src/NMC3D \
  --device /dev/dri:/dev/dri \
  nmc3d-noetic bash
```

Terminal 1, inside the container:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Terminal 2, rebuild and run calibration after every code change:

```bash
docker exec -it nmc3d bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws/src/NMC3D && cmake -S . -B build_nmc_docker && cmake --build build_nmc_docker --target calib -j$(nproc) && export LIBGL_ALWAYS_SOFTWARE=1 && ./build_nmc_docker/calib/calib ./Vocabulary/ORBvoc.txt config/sim/calib.yaml config/sim/front_cam.yaml config/sim/back_cam.yaml'
```

## 6. What To Look For

The NMC3D experimental code prints messages like:

```text
global final depth selection: valid ..., near ..., middle ..., far ..., selected ...
global 8x8 grid cells occupied: near ..., middle ..., far ...; selected near ..., middle ..., far ...
matched mps from CamMap matching size: ...
matched mps selected for optim size: ...
```

This means the current workflow was compiled and is running: CamMap keyframe matching
is unchanged, then one global final depth/grid selection is applied before optimization.

If it prints:

```text
local adaptive depth selection skipped: near ..., middle ..., far ...
```

then one depth group did not have enough matches, so the code kept the original CamMap
matches for that keyframe pair.

If it still prints the older messages:

```text
local quantile depth selection: valid ..., shallow ..., middle ..., deep ...
local adaptive depth selection: valid ..., near ..., middle ..., far ...
matched mps selected for global optim size: ...
matched mps all-depth for optim size: ...
```

then you are running a stale binary. Re-run the rebuild-and-run command in Section 5.

The final calibration output is still:

```text
inliers size: ...
---- first pose optim ----
euler: ...
trans: ...
---- final pose optim ----
euler: ...
trans: ...
```

Use these values to compare against the original `orbcalib-master` result.
