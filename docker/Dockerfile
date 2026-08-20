# syntax=docker/dockerfile:1
# Dev/build image for ORB_SLAM3_Modern (ubuntu:24.04, arm64-native on M-series).
# R1 toolchain bump (2026-08-20): 22.04 -> 24.04 = gcc-13.2 default, OpenCV
# 4.6.0, Boost 1.83. Feature-layer bit hashes were re-baselined for OpenCV 4.6
# (see benchmark/golden/bit_hashes_env.txt).
# R2 (2026-08-20): Eigen no longer comes from apt — the single Eigen is the
# third_party/eigen submodule (5.0.1), consumed live from the /workspace bind
# mount by our CMake (cmake/FindEigen3.cmake). Pangolin (v0.9.6 submodule) is
# baked below from named build contexts pointing at THIS repo's submodules
# (docker-compose additional_contexts: pangolinsrc, eigensrc). The Eigen copy
# Pangolin builds against is staged in /tmp of the pangolin stage only; the
# dev stage restarts FROM base and grafts just Pangolin's install tree, so
# the final image contains no Eigen outside /workspace — a consumer escaping
# the CMake pin fails loudly instead of silently using a second Eigen.
# The repository is bind-mounted at /workspace and built into the /build
# named volume by docker/scripts/container_build.sh.

FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git pkg-config ccache ca-certificates \
    libopencv-dev \
    libglew-dev libgl1-mesa-dev libepoxy-dev \
    libwayland-dev wayland-protocols libxkbcommon-dev \
    libboost-dev libboost-serialization-dev \
    libtbb-dev libgtk-3-dev libssl-dev \
    python3-dev python3-numpy \
    xvfb x11-apps \
    && rm -rf /var/lib/apt/lists/*

FROM base AS pangolin
# Eigen headers for Pangolin's build only (its components find_package(Eigen3
# NO_MODULE) needs an installed Eigen3Config). Header-only: configure+install,
# no compile. Same submodule bits our CMake pins at /workspace configure time.
COPY --from=eigensrc . /tmp/eigen
RUN cmake -S /tmp/eigen -B /tmp/eigen_build -Wno-dev \
      -DCMAKE_INSTALL_PREFIX=/tmp/eigen_install \
      -DBUILD_TESTING=OFF -DEIGEN_BUILD_TESTING=OFF \
      -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_DEMOS=OFF \
      -DEIGEN_BUILD_BLAS=OFF -DEIGEN_BUILD_LAPACK=OFF \
    && cmake --install /tmp/eigen_build
COPY --from=pangolinsrc . /tmp/Pangolin
RUN cmake -S /tmp/Pangolin -B /tmp/pangolin_build -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/tmp/eigen_install \
      -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
      -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    && cmake --build /tmp/pangolin_build -j8 \
    && cmake --install /tmp/pangolin_build \
    && rm -rf /tmp/Pangolin /tmp/pangolin_build /tmp/eigen /tmp/eigen_build /tmp/eigen_install

# dev restarts from base: only Pangolin's install tree is grafted in, the
# staged Eigen never reaches the final image (single-Eigen guarantee above).
FROM base AS dev
COPY --from=pangolin /usr/local/ /usr/local/
RUN ldconfig
ENV CCACHE_DIR=/ccache
WORKDIR /workspace
CMD ["bash"]
