# LingTu Navigation — build shortcuts

.PHONY: help build nav_kernel test clean install health benchmark format lint codegen-idl

.DEFAULT_GOAL := help
ROS_DISTRO ?= humble
ROS_SETUP ?= /opt/ros/$(ROS_DISTRO)/setup.bash

help:
	@echo "LingTu Navigation — available targets"
	@echo ""
	@echo "  Build:"
	@echo "    make build       - colcon build (full workspace, needs ROS2)"
	@echo "    make nav_kernel    - build LingTu native navigation kernel (no ROS2 needed)"
	@echo "    make codegen-idl - generate Python DDS types from IDL"
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
	@echo "    make navigation  - start navigation mode (lingtu.py nav)"
	@echo ""

build:
	@echo "Building workspace..."
	@bash -c "source '$(ROS_SETUP)' && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
	@echo "Done."

nav_kernel:
	@echo "Building LingTu native navigation kernel (nanobind, no ROS2 needed)..."
	@bash scripts/build/build_nav_kernel.sh

# Windows: .\scripts\codegen\run_codegen.ps1
codegen-idl:
	python scripts/codegen/idl_to_python.py src/message/idl/lingtu_slam.idl --output src/message/dds_types_generated/

build-debug:
	@echo "Building workspace (Debug)..."
	@bash -c "source '$(ROS_SETUP)' && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug"
	@echo "Done."

test:
	@echo "Running tests..."
	@bash -c "source '$(ROS_SETUP)' && source install/setup.bash && colcon test"
	@bash -c "source '$(ROS_SETUP)' && colcon test-result --verbose"

test-integration:
	@echo "Running integration tests..."
	@bash tests/integration/run_all.sh

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build/ install/ log/
	@echo "Done."

install:
	@echo "Installing systemd services..."
	@bash scripts/deploy/s100p/install_services.sh
	@echo "Done."

health:
	@python3 lingtu.py health

benchmark:
	@bash tests/benchmark/run_all.sh

format:
	@echo "Formatting C++ sources..."
	@find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
	@echo "Done."

lint:
	@echo "Running clang-tidy..."
	@bash -c "source '$(ROS_SETUP)' && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
	@find src -name "*.cpp" | xargs clang-tidy -p build/

py-lint:
	ruff check src/ cli/

py-format:
	ruff format src/ cli/

py-fix:
	ruff check --fix src/ cli/
	@echo "Done."

mapping:
	@bash -c "source '$(ROS_SETUP)' && source install/setup.bash && python3 lingtu.py map"

navigation:
	@bash -c "source '$(ROS_SETUP)' && source install/setup.bash && python3 lingtu.py nav"

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
