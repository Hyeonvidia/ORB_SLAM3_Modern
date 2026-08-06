# syntax=docker/dockerfile:1
# Dev/build image for ORB_SLAM3_Modern (ubuntu:22.04, arm64-native on M-series).
# Pangolin is baked from the pinned third_party/Pangolin submodule (build
# context "pangolinsrc"); the repository itself is bind-mounted at /workspace
# and built into the /build named volume by docker/scripts/container_build.sh.

FROM ubuntu:22.04 AS base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git pkg-config ccache ca-certificates \
    libeigen3-dev libopencv-dev \
    libglew-dev libgl1-mesa-dev libepoxy-dev \
    libwayland-dev wayland-protocols libxkbcommon-dev \
    libboost-dev libboost-serialization-dev \
    libtbb-dev libgtk-3-dev libssl-dev \
    python3-dev python3-numpy \
    xvfb x11-apps \
    && rm -rf /var/lib/apt/lists/*

FROM base AS pangolin
COPY --from=pangolinsrc . /tmp/Pangolin
RUN cmake -S /tmp/Pangolin -B /tmp/pangolin_build -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
      -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    && cmake --build /tmp/pangolin_build -j8 \
    && cmake --install /tmp/pangolin_build \
    && ldconfig \
    && rm -rf /tmp/Pangolin /tmp/pangolin_build

FROM pangolin AS dev
ENV CCACHE_DIR=/ccache
WORKDIR /workspace
CMD ["bash"]
