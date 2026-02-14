# MapPilot 测试框架

本目录包含 MapPilot 3D NAV 系统的所有测试。

## 📁 目录结构

```
tests/
├── benchmark/          # 性能基准测试
│   ├── run_all.sh
│   ├── benchmark_slam.sh
│   ├── benchmark_planner.sh
│   └── benchmark_grpc.sh
├── integration/        # 集成测试
│   ├── run_all.sh
│   ├── test_full_stack.sh
│   ├── test_grpc_endpoints.py
│   ├── test_topic_hz.py
│   └── test_network_failure.py
└── e2e/               # 端到端测试
    ├── test_mapping_flow.py
    ├── test_navigation_flow.py
    └── test_ota_flow.py
```

## 🚀 快速开始

### 运行所有测试

```bash
# 从项目根目录
make test
```

### 运行性能基准测试

```bash
make benchmark
```

### 运行集成测试

```bash
make test-integration
```

## 📊 性能基准测试

性能基准测试用于建立系统性能基线，用于回归测试。

### benchmark_slam.sh
测试 FAST-LIO2 的处理速度和资源占用。

**前置条件**:
- 需要 rosbag 测试数据（放在 `~/rosbags/`）
- ROS 2 环境已配置

**输出指标**:
- 处理时间
- CPU 使用率
- 内存使用率

### benchmark_planner.sh
测试 PCT Planner 的规划速度。

**前置条件**:
- 需要地图数据（`src/global_planning/PCT_planner/rsc/tomogram/`）

**输出指标**:
- 平均规划时间
- 规划成功率

### benchmark_grpc.sh
测试 gRPC Gateway 的吞吐量和延迟。

**前置条件**:
- gRPC Gateway 正在运行（端口 50051）
- 可选：安装 grpcurl 进行完整测试

**输出指标**:
- QPS (每秒请求数)
- 平均延迟
- 并发性能

### 查看历史结果

```bash
ls -lh tests/benchmark/results/
cat tests/benchmark/results/benchmark_20260214_*.txt
```

## 🧪 集成测试

集成测试验证系统各模块协同工作。

### test_full_stack.sh
验证所有关键节点能否正常启动。

**检查项**:
- 关键 ROS 2 节点
- 关键话题
- 系统启动时间

### test_grpc_endpoints.py
测试所有关键 gRPC 端点。

**测试的 RPC**:
- GetSystemInfo
- AcquireLease / ReleaseLease
- GetMode
- ListMaps
- GetInstalledVersions

### test_topic_hz.py
验证关键话题以预期频率发布。

**监控话题**:
- `/nav/odometry` (期望 10 Hz)
- `/nav/terrain_map` (期望 1 Hz)
- `/nav/path` (期望 1 Hz)
- `/cmd_vel` (期望 10 Hz)

### test_network_failure.py
模拟网络故障，测试系统容错能力。

**测试场景**:
- 网络断开
- 高延迟
- 丢包

## 🎯 端到端测试

端到端测试验证完整的用户流程。

### test_mapping_flow.py
测试完整的建图流程。

**流程**:
1. 启动建图模式
2. 移动机器人
3. 保存地图
4. 验证地图文件

### test_navigation_flow.py
测试完整的导航流程。

**流程**:
1. 启动导航模式
2. 设置目标点
3. 执行导航
4. 验证到达目标

### test_ota_flow.py
测试完整的 OTA 更新流程。

**流程**:
1. 下载更新包
2. 验证签名
3. 安装更新
4. 健康检查
5. 回滚测试

## 📝 编写新测试

### 集成测试模板

```bash
#!/bin/bash
# 测试描述

set -e

echo "测试名称"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. 准备环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. 执行测试
# ... 测试逻辑 ...

# 3. 验证结果
if [ 测试条件 ]; then
    echo "✅ 测试通过"
    exit 0
else
    echo "❌ 测试失败"
    exit 1
fi
```

### Python 测试模板

```python
#!/usr/bin/env python3
"""测试描述"""

import unittest
import rclpy

class TestExample(unittest.TestCase):
    def setUp(self):
        """测试前准备"""
        rclpy.init()

    def tearDown(self):
        """测试后清理"""
        rclpy.shutdown()

    def test_something(self):
        """测试某个功能"""
        # 测试逻辑
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
```

## 🔧 故障排查

### 测试失败常见原因

1. **ROS 2 环境未配置**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

2. **系统未启动**
   ```bash
   make navigation  # 或 make mapping
   ```

3. **缺少测试数据**
   - rosbag 数据: `~/rosbags/`
   - 地图数据: `src/global_planning/PCT_planner/rsc/tomogram/`

4. **端口被占用**
   ```bash
   # 检查 gRPC Gateway 端口
   netstat -tuln | grep 50051
   ```

## 📊 测试覆盖率

当前测试覆盖率约 40%，目标 60%+。

**已覆盖模块**:
- ✅ remote_monitoring (gtest)
- ✅ Flutter 客户端 (56 tests)

**待覆盖模块**:
- 🔲 PCT Planner
- 🔲 LocalPlanner
- 🔲 ModeManager
- 🔲 OTA 流程

## 🎯 测试最佳实践

1. **每次提交前运行测试**
   ```bash
   make test
   ```

2. **定期运行性能基准测试**
   ```bash
   make benchmark
   ```

3. **CI/CD 自动运行测试**
   - GitHub Actions 已配置自动测试

4. **保持测试独立**
   - 每个测试应该能独立运行
   - 不依赖其他测试的状态

5. **测试应该快速**
   - 单元测试 < 1s
   - 集成测试 < 30s
   - 端到端测试 < 5min

## 📚 参考资料

- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [gRPC Testing Best Practices](https://grpc.io/docs/guides/testing/)
- [Python unittest Documentation](https://docs.python.org/3/library/unittest.html)

---

**最后更新**: 2026-02-14
