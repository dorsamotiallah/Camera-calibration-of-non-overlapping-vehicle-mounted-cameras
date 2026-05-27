FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config python3-pip python3-catkin-tools \
    libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
    libglew-dev libepoxy-dev libgl1-mesa-dev libegl1-mesa-dev \
    libpython3-dev ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev \
    libjpeg-dev libpng-dev libtiff5-dev \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-sensor-msgs ros-noetic-tf \
    x11-apps mesa-utils \
 && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin && \
    cd /tmp/Pangolin && \
    git checkout v0.6 && \
    cmake -S /tmp/Pangolin -B /tmp/Pangolin/build \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTS=OFF \
      -DBUILD_TOOLS=OFF \
      -DCMAKE_CXX_FLAGS="-Wno-error" && \
    cmake --build /tmp/Pangolin/build -j"$(nproc)" && \
    cmake --install /tmp/Pangolin/build && \
    rm -rf /tmp/Pangolin

RUN python3 -m pip install --no-cache-dir mcap-ros2-support numpy

WORKDIR /ws
