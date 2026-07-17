FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake \
    libpcl-dev libopencv-dev libceres-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

# Copy original livox_camera_calib (header-only dependency)
COPY livox_camera_calib/include /src/livox_camera_calib/include

# Copy standalone project
COPY livox_calib_standalone /src/livox_calib_standalone

RUN cd /src/livox_calib_standalone && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . -j$(nproc)

CMD ["/src/livox_calib_standalone/build/livox_calib", "--help"]
