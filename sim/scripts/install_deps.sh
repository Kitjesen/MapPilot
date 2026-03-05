#!/bin/bash
# MapPilot sim/ 依赖安装脚本
# 在机器人或开发服务器上执行
set -e

echo "=== [1/4] 安装 Python 依赖 ==="
pip install mujoco numpy scipy

echo "=== [2/4] 编译 mujoco_ray_caster C++ plugin ==="
MUJOCO_SRC_DIR=${1:-/tmp/mujoco_src}

if [ ! -d "$MUJOCO_SRC_DIR" ]; then
    git clone https://github.com/google-deepmind/mujoco.git "$MUJOCO_SRC_DIR"
fi

cd "$MUJOCO_SRC_DIR/plugin"
if [ ! -d "mujoco_ray_caster" ]; then
    git clone https://github.com/Albusgive/mujoco_ray_caster.git
fi

# 修改 CMakeLists.txt
if ! grep -q "mujoco_ray_caster" "$MUJOCO_SRC_DIR/CMakeLists.txt"; then
    echo 'add_subdirectory(plugin/mujoco_ray_caster)' >> "$MUJOCO_SRC_DIR/CMakeLists.txt"
    echo "  Added mujoco_ray_caster to CMakeLists.txt"
fi

cd "$MUJOCO_SRC_DIR"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# 安装 plugin .so
cd bin && mkdir -p mujoco_plugin
cp ../lib/libray_caster*.so ./mujoco_plugin/ 2>/dev/null || \
cp ../lib/libmujoco_ray_caster*.so ./mujoco_plugin/ 2>/dev/null || true

PLUGIN_PATH="$(pwd)/mujoco_plugin"
echo "  Plugin .so installed to: $PLUGIN_PATH"
echo "  Add to your shell:"
echo "    export MUJOCO_PLUGIN_PATH=$PLUGIN_PATH"

echo ""
echo "=== [3/4] 可选：OmniPerception (GPU, Livox 精确模式) ==="
echo "  需要 CUDA 11+ 和 RTX GPU"
echo "  pip install warp-lang[extras] taichi"
echo "  git clone https://github.com/aCodeDog/OmniPerception.git"
echo "  cd OmniPerception/LidarSensor && pip install -e ."

echo ""
echo "=== [4/4] ROS2 依赖 ==="
echo "  source /opt/ros/humble/setup.bash"
echo "  sudo apt install -y ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-tf2-ros"
echo "  注意: 使用 cyclonedds-cpp，fastdds 可能有问题"
echo "    sudo apt install ros-humble-rmw-cyclonedds-cpp"
echo "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

echo ""
echo "=== 完成 ==="
echo "测试: python sim/scripts/run_sim.py --world open_field --no-ros"
