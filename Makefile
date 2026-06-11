# LingTu Navigation — build shortcuts

.PHONY: help build nav_core test clean install health benchmark format lint

.DEFAULT_GOAL := help

help:
	@echo "LingTu Navigation — available targets"
	@echo ""
	@echo "  Build:"
	@echo "    make build       - colcon build (full workspace, needs ROS2)"
	@echo "    make nav_core    - build _nav_core.so only (no ROS2 needed)"
	@echo "    make build-debug - colcon build in Debug mode"
	@echo ""
	@echo "  Test:"
	@echo "    make test        - colcon test"
	@echo "    make test-integration - integration tests"
	@echo ""
	@echo "  Ops:"
	@echo "    make install     - install systemd services"
	@echo "    make health      - system health check"
	@echo "    make clean       - remove build/ install/ log/"
	@echo ""
	@echo "  Code quality:"
	@echo "    make format      - clang-format all C++ sources"
	@echo "    make lint        - clang-tidy"
	@echo ""
	@echo "  Launch (Module-First profiles via lingtu.py):"
	@echo "    make mapping     - start mapping mode (lingtu.py map)"
	@echo "    make navigation  - start navigation mode (lingtu.py s100p)"
	@echo ""

build:
	@echo "Building workspace..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
	@echo "Done."

nav_core:
	@echo "Building _nav_core.so (nanobind, no ROS2 needed)..."
	@bash scripts/build_nav_core.sh

build-debug:
	@echo "Building workspace (Debug)..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug"
	@echo "Done."

test:
	@echo "Running tests..."
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test"
	@bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"

test-integration:
	@echo "Running integration tests..."
	@bash tests/integration/run_all.sh

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build/ install/ log/
	@echo "Done."

install:
	@echo "Installing systemd services..."
	@bash scripts/install_services.sh
	@echo "Done."

health:
	@bash scripts/health_check.sh

benchmark:
	@bash tests/benchmark/run_all.sh

format:
	@echo "Formatting C++ sources..."
	@find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
	@echo "Done."

lint:
	@echo "Running clang-tidy..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
	@find src -name "*.cpp" | xargs clang-tidy -p build/

py-lint:
	ruff check src/ cli/

py-format:
	ruff format src/ cli/

py-fix:
	ruff check --fix src/ cli/
	@echo "Done."

mapping:
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 lingtu.py map"

navigation:
	@bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 lingtu.py s100p"

sync-version:
	@bash scripts/deploy/sync_versions.sh

docker-build:
	@docker build -f docker/Dockerfile -t mappilot-nav:latest .

docker-run:
	@docker-compose up -d

docker-stop:
	@docker-compose down

docs:
	@doxygen Doxyfile 2>/dev/null || echo "Doxygen not installed, skipping."

check: build test health
