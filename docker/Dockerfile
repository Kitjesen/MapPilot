# =============================================================================
# LingTu Navigation System — Multi-stage Dockerfile
#
# Targets (selected via `--target`):
#   build   — Build C++ nav_core shared library and prepare colcon workspace
#   runtime — Production runtime image (ROS2 Humble + LingTu)
#   test    — Test image (runtime + pytest)
#
# Usage:
#   docker build --target runtime -t lingtu:latest .
#   docker build --target test -t lingtu:test .
# =============================================================================

# ── Stage 0: Python dependencies ──────────────────────────────────────────
FROM ros:humble AS python-deps
WORKDIR /deps
COPY requirements.txt .
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir -r requirements.txt

# ── Stage 1: Build C++ nav_core ──────────────────────────────────────────
FROM ros:humble AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake build-essential python3-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /build
COPY src/nav/core/ /build/
RUN mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# ── Stage 2: Build stage (for colcon builds in CI) ───────────────────────
FROM ros:humble AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake build-essential python3-pip python3-dev \
    python3-colcon-common-extensions python3-rosdep \
    libpcl-dev libeigen3-dev libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . .
RUN pip install --no-cache-dir -r requirements.txt
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ── Stage 3: Runtime ─────────────────────────────────────────────────────
FROM ros:humble-ros-base AS runtime

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy nav_core shared library from builder
COPY --from=builder /build/build/*.so /usr/local/lib/
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# Copy Python dependencies layer
COPY --from=python-deps /usr/local/lib/python3.*/dist-packages/ /usr/local/lib/python3.*

# Copy application code
WORKDIR /app
COPY . .

# Expose gateway (HTTP/WS/SSE) and MCP ports
EXPOSE 5050 8090

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD python3 -c "import requests; requests.get('http://localhost:5050/api/v1/health', timeout=5)" || exit 1

# Default command: stub profile (no hardware required)
CMD ["python3", "lingtu.py", "stub"]

# ── Stage 4: Test ────────────────────────────────────────────────────────
FROM runtime AS test

# Install test dependencies
RUN pip install --no-cache-dir pytest pytest-asyncio pytest-mock pytest-benchmark

# Run framework tests
CMD ["python3", "-m", "pytest", "src/core/tests/", "-q", "-x", "--timeout=60"]
