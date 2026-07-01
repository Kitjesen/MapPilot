#!/bin/bash
# 安装 gRPC 和 Protobuf 依赖

set -e

echo "Installing gRPC dependencies for ROS2..."

# 更新包列表
sudo apt-get update

# 安装基础依赖
sudo apt-get install -y \
    build-essential \
    autoconf \
    libtool \
    pkg-config \
    cmake \
    git

# 安装 protobuf
sudo apt-get install -y \
    libprotobuf-dev \
    protobuf-compiler

# 安装 gRPC
# 方式 1：尝试从包管理器安装（Ubuntu 20.04+）
if sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc 2>/dev/null; then
    echo "gRPC installed from apt"
    grpc_cpp_plugin --version
else
    echo "apt install failed, trying alternative..."
    
    # 方式 2：从源码编译（如果包管理器没有）
    echo "Building gRPC from source (this may take 30+ minutes)..."
    
    GRPC_VERSION="v1.54.0"
    BUILD_DIR="/tmp/grpc_build"
    
    rm -rf $BUILD_DIR
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR
    
    git clone --recurse-submodules -b $GRPC_VERSION --depth 1 \
        https://github.com/grpc/grpc
    
    cd grpc
    mkdir -p cmake/build
    cd cmake/build
    
    cmake -DgRPC_INSTALL=ON \
          -DgRPC_BUILD_TESTS=OFF \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          ../..
    
    make -j$(nproc)
    sudo make install
    
    echo "gRPC built and installed to /usr/local"
    cd ~
    rm -rf $BUILD_DIR
fi

# 验证安装
echo "Verifying installation..."
protoc --version
grpc_cpp_plugin --version || echo "Warning: grpc_cpp_plugin not in PATH, may need to set manually"

echo "gRPC installation complete!"
