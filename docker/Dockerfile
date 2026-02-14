# ═══════════════════════════════════════════════════════════════
# 3d_NAV Navigation Stack — Multi-stage Docker Build
#
# 架构:
#   Stage 1 (build)  — 完整编译环境, 产出 install/ 目录
#   Stage 2 (runtime) — 最小运行时, 仅包含二进制和动态库
#
# 构建:
#   docker build -f docker/Dockerfile -t 3dnav:latest .
#
# 运行 (见 docker-compose.yml):
#   docker compose up nav-stack
# ═══════════════════════════════════════════════════════════════

# ── ARG: 可在 CI 中覆盖 ──
ARG ROS_DISTRO=humble
ARG UBUNTU_VERSION=22.04

# ╔══════════════════════════════════════════════════╗
# ║  Stage 1: BUILD                                  ║
# ╚══════════════════════════════════════════════════╝
FROM ros:${ROS_DISTRO} AS build

SHELL ["/bin/bash", "-c"]

# 避免交互式 tzdata 等提示
ENV DEBIAN_FRONTEND=noninteractive

# ── 系统依赖 ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 编译工具
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    # 库依赖
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libgrpc++-dev \
    libssl-dev \
    protobuf-compiler-grpc \
    # PCT planner 依赖
    python3-pybind11 \
    libboost-all-dev \
    # gRPC C++ runtime
    libgrpc-dev \
    # 其他
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── 复制源码 ──
WORKDIR /ws
COPY . /ws/

# ── 构建 PCT Planner C++ Core ──
RUN cd src/global_planning/PCT_planner/planner && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    cd /ws

# ── colcon 构建 ──
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers $(nproc)

# ── 构建 OTA Daemon ──
RUN cd src/ota_daemon && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/ws/install/ota_daemon && \
    make -j$(nproc) install

# ── 运行测试 (可选, 通过 --target test 阶段控制) ──
FROM build AS test
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /ws/install/setup.bash && \
    colcon test \
      --packages-select remote_monitoring \
      --return-code-on-test-failure && \
    colcon test-result --verbose

# ╔══════════════════════════════════════════════════╗
# ║  Stage 2: RUNTIME                               ║
# ╚══════════════════════════════════════════════════╝
FROM ros:${ROS_DISTRO}-ros-core AS runtime

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ── 运行时依赖 (仅动态库, 不含 -dev 头文件) ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 运行时
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    # 库运行时
    libpcl-io1.12 \
    libpcl-filters1.12 \
    libpcl-registration1.12 \
    libopencv-core4.5d \
    libopencv-imgcodecs4.5d \
    libopencv-imgproc4.5d \
    libgrpc++1 \
    libssl3 \
    # 进程管理
    supervisor \
    # 工具
    iproute2 \
    iputils-ping \
    procps \
    && rm -rf /var/lib/apt/lists/*

# ── 创建运行时用户和目录 ──
RUN useradd -m -s /bin/bash sunrise && \
    mkdir -p /opt/nav /opt/robot/logs /opt/robot/flight_records \
             /opt/robot/ota /opt/robot/bags /etc/nav && \
    chown -R sunrise:sunrise /opt/robot

# ── 从 build 阶段复制产物 ──
COPY --from=build /ws/install /opt/nav/install
COPY --from=build /ws/src/ota_daemon/config /opt/nav/ota_config

# ── 复制运行时配置文件 ──
COPY launch/        /opt/nav/launch/
COPY scripts/       /opt/nav/scripts/
COPY maps/          /opt/nav/maps/

# ── 复制 Docker 入口脚本和 supervisord 配置 ──
COPY docker/entrypoint.sh      /entrypoint.sh
COPY docker/supervisord.conf   /etc/supervisor/conf.d/nav.conf
RUN chmod +x /entrypoint.sh

# ── 环境变量 ──
ENV ROS_DISTRO=${ROS_DISTRO} \
    NAV_DIR=/opt/nav \
    NAV_LOG_DIR=/opt/robot/logs \
    NAV_MAP_DIR=/opt/nav/maps \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///nav_ws/config/cyclonedds.xml \
    # DDS: 禁用共享内存 (容器间不可用)
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/nav/config/fastdds_no_shm.xml \
    LANG=en_US.UTF-8

# ── 端口 ──
EXPOSE 50051 50052

# ── 健康检查 ──
HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
    CMD bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
                 source /opt/nav/install/setup.bash && \
                 ros2 node list 2>/dev/null | grep -q grpc_gateway' || exit 1

# ── 入口 ──
ENTRYPOINT ["/entrypoint.sh"]
CMD ["supervisord", "-c", "/etc/supervisor/conf.d/nav.conf"]
