# syntax=docker/dockerfile:1
# pyorbslam3 — self-contained image: Pangolin v0.9 + ORB-SLAM3 (patched) + Python bindings.
#
#   docker build -t pyorbslam3 .                       # ~30-60 min, needs ~6 GB RAM at JOBS=3
#   docker run --rm -it -v /path/to/datasets:/data pyorbslam3 \
#       python3 examples/benchmark.py --dataset tumvi --root /data/dataset-room1_512_16 --mode stereo-imu
#
# Build parallelism is capped (ARG JOBS) because a few ORB-SLAM3 translation
# units need ~2 GB each to compile.

ARG UBUNTU=22.04

# ---------------------------------------------------------------------------
FROM ubuntu:${UBUNTU} AS builder
ARG JOBS=3
ARG ORBSLAM3_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
ARG PANGOLIN_VERSION=0.9
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git ca-certificates pkg-config wget \
      python3-dev python3-pip python3-numpy \
      libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev \
      libglew-dev libgl1-mesa-dev libegl1-mesa-dev libglu1-mesa-dev \
      libwayland-dev libxkbcommon-dev wayland-protocols \
      libx11-dev libxext-dev libsm-dev libice-dev \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir "cmake>=3.25,<4" "pybind11>=2.10"   # ORB-SLAM3 needs CMake >= 3.25

# ---- Pangolin (static, PIC, viewer only) ----------------------------------
RUN wget -qO- https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v${PANGOLIN_VERSION}.tar.gz \
      | tar xz -C /opt \
    && cmake -S /opt/Pangolin-${PANGOLIN_VERSION} -B /opt/Pangolin-${PANGOLIN_VERSION}/build \
         -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
         -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF \
    && cmake --build /opt/Pangolin-${PANGOLIN_VERSION}/build -j${JOBS} \
    && cmake --install /opt/Pangolin-${PANGOLIN_VERSION}/build \
    && rm -rf /opt/Pangolin-${PANGOLIN_VERSION}

# ---- ORB-SLAM3 (pinned commit + patches) ----------------------------------
COPY docker/patches /tmp/patches
RUN git init -q /opt/ORB_SLAM3 && cd /opt/ORB_SLAM3 \
    && git remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
    && git fetch -q --depth 1 origin ${ORBSLAM3_COMMIT} \
    && git checkout -q FETCH_HEAD \
    && git apply --whitespace=nowarn /tmp/patches/0001-*.patch /tmp/patches/0002-*.patch \
    && rm -rf .git Examples_old Examples/*/TUM_IMU \
    && tar -xzf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
RUN cmake -S /opt/ORB_SLAM3 -B /opt/ORB_SLAM3/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/ORB_SLAM3/build -j${JOBS}

# ---- pyorbslam3 (PEP 517 build against the tree above) ---------------------
COPY . /src/pyorbslam3
RUN cd /src/pyorbslam3 \
    && ORBSLAM3_ROOT=/opt/ORB_SLAM3 pip3 install --no-cache-dir --no-deps --target /opt/pyinstall . \
    && python3 -c "import sys; sys.path.insert(0, '/opt/pyinstall'); import pyorbslam3"

# ---------------------------------------------------------------------------
FROM ubuntu:${UBUNTU}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-numpy python3-opencv \
      libboost-serialization1.74.0 libssl3 libglew2.2 libgl1 libglx0 libopengl0 libegl1 libglu1-mesa \
      libwayland-client0 libwayland-egl1 libwayland-cursor0 libxkbcommon0 \
      libx11-6 libxext6 libsm6 libice6 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/pyinstall/ /usr/local/lib/python3.10/dist-packages/
COPY --from=builder /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt /opt/pyorbslam3/vocabulary/ORBvoc.txt
COPY configs /opt/pyorbslam3/configs
COPY examples /opt/pyorbslam3/examples
COPY README.md /opt/pyorbslam3/README.md

WORKDIR /opt/pyorbslam3
ENV PYORBSLAM3_VOCAB=/opt/pyorbslam3/vocabulary/ORBvoc.txt
RUN python3 -c "import pyorbslam3; print('pyorbslam3 OK:', [s.name for s in pyorbslam3.Sensor.__members__.values()])"
CMD ["python3", "-c", "import pyorbslam3; print('pyorbslam3 ready; see examples/benchmark.py --help')"]
