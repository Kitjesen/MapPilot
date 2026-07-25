# LingTu native Product build shortcuts

.PHONY: help build build-debug nav_kernel codegen-idl test test-python test-native clean install health benchmark format lint py-lint py-format py-fix mapping navigation sync-version docker-build docker-run docker-stop docs check

.DEFAULT_GOAL := help

PYTHON ?= python3
BUILD_TYPE ?= Release
NATIVE_TEST_BUILD_DIR ?= build/nav-cpp

help:
	@echo "LingTu — available targets"
	@echo ""
	@echo "  Build:"
	@echo "    make build       - build the native field Product ($(BUILD_TYPE))"
	@echo "    make build-debug - build the native field Product in Debug mode"
	@echo "    make nav_kernel  - build the native navigation Python extension"
	@echo "    make codegen-idl - generate Python DDS types from IDL"
	@echo ""
	@echo "  Test:"
	@echo "    make test        - run Python and portable native C++ tests"
	@echo "    make test-python - run the configured pytest suite"
	@echo "    make test-native - build and run portable navigation C++ tests"
	@echo ""
	@echo "  Ops:"
	@echo "    make install     - install the canonical native field services"
	@echo "    make mapping     - switch ProductControl to the map Product"
	@echo "    make navigation MAP=<name> - switch to nav with a saved map"
	@echo "    make health      - run the system health check"
	@echo "    make clean       - remove local build artifacts"
	@echo ""
	@echo "  Code quality:"
	@echo "    make format      - format C++ sources"
	@echo "    make lint        - run Python lint checks"
	@echo ""

build:
	@echo "Building native field Product ($(BUILD_TYPE))..."
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" bash scripts/build/build_native_runtime.sh
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON bash scripts/build/build_livox_sdk2_stream.sh
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" LINGTU_SLAM_BUILD_DDS_RUNTIME=ON LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF bash scripts/build/build_slam_core.sh
	@if [ -f scripts/build/build_mapd.sh ]; then CMAKE_BUILD_TYPE="$(BUILD_TYPE)" bash scripts/build/build_mapd.sh; fi
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" bash scripts/build/build_dds_probe.sh
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" bash scripts/build/build_nav_endpoint.sh
	@CMAKE_BUILD_TYPE="$(BUILD_TYPE)" bash scripts/build/build_driver.sh
	@echo "Native field Product build complete."

build-debug:
	@$(MAKE) build BUILD_TYPE=Debug

nav_kernel:
	@echo "Building LingTu native navigation kernel..."
	@bash scripts/build/build_nav_kernel.sh

# Windows: .\scripts\codegen\run_codegen.ps1
codegen-idl:
	@$(PYTHON) scripts/codegen/idl_to_python.py src/message/idl/lingtu_slam.idl --output src/message/dds_types_generated/

test: test-python test-native

test-python:
	@$(PYTHON) -m pytest

test-native:
	@cmake -S src/nav/cpp -B "$(NATIVE_TEST_BUILD_DIR)" -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" -DLINGTU_NAV_CPP_BUILD_TESTS=ON -DLINGTU_NAV_CPP_BUILD_ENDPOINT=OFF -DLINGTU_NAV_CPP_BUILD_PYTHON=OFF
	@cmake --build "$(NATIVE_TEST_BUILD_DIR)" --parallel
	@ctest --test-dir "$(NATIVE_TEST_BUILD_DIR)" --output-on-failure

clean:
	@echo "Cleaning local build artifacts..."
	@rm -rf build/ install/ log/
	@echo "Done."

install:
	@echo "Installing canonical native field services..."
	@bash scripts/deploy/thunder/install_services.sh field-cpp
	@echo "Done."

health:
	@$(PYTHON) lingtu.py health

benchmark: nav_kernel
	@$(PYTHON) tests/benchmark/benchmark_local_planner.py

format:
	@echo "Formatting C++ sources..."
	@find src -type f \( -name "*.cpp" -o -name "*.hpp" \) -print0 | xargs -0 clang-format -i
	@echo "Done."

lint: py-lint

py-lint:
	@ruff check src/ cli/ tests/

py-format:
	@ruff format src/ cli/ tests/

py-fix:
	@ruff check --fix src/ cli/ tests/

mapping:
	@$(PYTHON) -m lingtu.control switch map --json

navigation:
	$(if $(strip $(MAP)),,$(error MAP is required; use: make navigation MAP=<saved-map>))
	@$(PYTHON) -m lingtu.control switch nav --map "$(MAP)" --json

sync-version:
	@bash scripts/deploy/sync_versions.sh

docker-build:
	@docker build -f docker/Dockerfile -t lingtu:latest .

docker-run:
	@docker-compose up -d

docker-stop:
	@docker-compose down

docs:
	@doxygen Doxyfile 2>/dev/null || echo "Doxygen not installed, skipping."

check: build test lint health
