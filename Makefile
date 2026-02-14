# MapPilot 3D NAV - 快捷命令
# 用途: 简化常用操作，提升开发效率

.PHONY: help build test clean install health benchmark format lint

# 默认目标
.DEFAULT_GOAL := help

help:
	@echo "=========================================="
	@echo "  MapPilot 3D NAV - 可用命令"
	@echo "=========================================="
	@echo ""
	@echo "构建与测试:"
	@echo "  make build      - 编译整个工作空间"
	@echo "  make test       - 运行所有测试"
	@echo "  make clean      - 清理编译产物"
	@echo ""
	@echo "部署与运维:"
	@echo "  make install    - 安装 systemd 服务"
	@echo "  make health     - 系统健康检查"
	@echo "  make benchmark  - 性能基准测试"
	@echo ""
	@echo "代码质量:"
	@echo "  make format     - 格式化代码 (clang-format)"
	@echo "  make lint       - 代码检查 (clang-tidy)"
	@echo ""
	@echo "快速启动:"
	@echo "  make mapping    - 启动建图模式"
	@echo "  make navigation - 启动导航模式"
	@echo ""

build:
	@echo "🔨 编译工作空间..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
	@echo "✅ 编译完成"

build-debug:
	@echo "🔨 编译工作空间 (Debug 模式)..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug"
	@echo "✅ 编译完成"

test:
	@echo "🧪 运行测试..."
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test"
	@bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"

test-integration:
	@echo "🧪 运行集成测试..."
	@bash tests/integration/run_all.sh

clean:
	@echo "🧹 清理编译产物..."
	@rm -rf build/ install/ log/
	@echo "✅ 清理完成"

install:
	@echo "📦 安装系统服务..."
	@bash scripts/install_services.sh
	@echo "✅ 安装完成"

health:
	@echo "🏥 系统健康检查..."
	@bash scripts/health_check.sh

benchmark:
	@echo "⚡ 性能基准测试..."
	@bash tests/benchmark/run_all.sh

format:
	@echo "🎨 格式化代码..."
	@find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
	@echo "✅ 格式化完成"

lint:
	@echo "🔍 代码检查..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
	@find src -name "*.cpp" | xargs clang-tidy -p build/
	@echo "✅ 检查完成"

mapping:
	@echo "🗺️  启动建图模式..."
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch/navigation_bringup.launch.py"

navigation:
	@echo "🚀 启动导航模式..."
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch/navigation_run.launch.py"

# 版本同步
sync-version:
	@echo "🔄 同步版本号..."
	@bash scripts/sync_versions.sh
	@echo "✅ 版本同步完成"

# Docker 相关
docker-build:
	@echo "🐳 构建 Docker 镜像..."
	@docker build -f docker/Dockerfile -t mappilot-nav:latest .
	@echo "✅ Docker 镜像构建完成"

docker-run:
	@echo "🐳 启动 Docker 容器..."
	@docker-compose up -d
	@echo "✅ Docker 容器已启动"

docker-stop:
	@echo "🐳 停止 Docker 容器..."
	@docker-compose down
	@echo "✅ Docker 容器已停止"

# 文档生成
docs:
	@echo "📚 生成文档..."
	@doxygen Doxyfile 2>/dev/null || echo "⚠️  Doxygen 未安装，跳过文档生成"
	@echo "✅ 文档生成完成"

# 快速检查（编译 + 测试 + 健康检查）
check: build test health
	@echo "✅ 所有检查通过"
