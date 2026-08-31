# =============================================================================
# LingTu native multi-stage image
#
# Targets:
#   build   - compile and test the native DDS field binaries
#   runtime - Python Host/Gateway plus the compiled native artifacts
#   test    - runtime image with the project's development test extra
# =============================================================================

ARG UBUNTU_VERSION=22.04

FROM ubuntu:${UBUNTU_VERSION} AS build

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cargo \
    cmake \
    cyclonedds-dev \
    cyclonedds-tools \
    libeigen3-dev \
    libgrpc++-dev \
    libgtest-dev \
    liboctomap-dev \
    libopencv-dev \
    libpcl-dev \
    libprotobuf-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    protobuf-compiler-grpc \
    python3 \
    python3-dev \
    python3-pip \
    rustc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN python3 -c "import sys; assert sys.version_info[:2] == (3, 10), sys.version" \
    && python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && python3 -m pip install --no-cache-dir '.[dev]'

ENV CMAKE_BUILD_TYPE=Release \
    LINGTU_BUILD_JOBS=4 \
    LINGTU_NAV_CPP_BUILD_TESTS=ON \
    LINGTU_NAV_ENDPOINT_RUN_TESTS=ON \
    LINGTU_SLAM_BUILD_DDS_RUNTIME=ON \
    LINGTU_SLAM_BUILD_TESTS=ON

RUN make build PYTHON=python3

FROM ubuntu:${UBUNTU_VERSION} AS runtime

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Development packages are intentionally installed here as a portable way to
# provide every shared library needed by the native artifacts on Ubuntu 22.04.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    cyclonedds-dev \
    libgrpc++-dev \
    liboctomap-dev \
    libpcl-dev \
    libprotobuf-dev \
    libyaml-cpp-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .
COPY --from=build /app/build /app/build

RUN python3 -c "import sys; assert sys.version_info[:2] == (3, 10), sys.version" \
    && python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && python3 -m pip install --no-cache-dir '.[gateway]'

ENV PYTHONPATH=/app/src:/app \
    PATH=/app/build/nav_endpoint:/app/build/slam_core:/app/build/maps:/app/build/dds_probe:/app/build/driver:/app/build/livox_sdk2_stream:${PATH} \
    LD_LIBRARY_PATH=/app/build/nav_endpoint:/app/build/slam_core:/app/build/maps:/app/build/driver:/app/build/livox_sdk2_stream:/usr/local/lib

FROM runtime AS test

RUN python3 -m pip install --no-cache-dir '.[dev]'

CMD ["python3", "-m", "pytest", "src/runtime/tests/", "-q", "-x"]
