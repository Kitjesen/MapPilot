#!/usr/bin/env python3
"""
gRPC 接口集成测试
测试所有关键 RPC 端点的功能和性能
"""

import grpc
import sys
import time
from typing import List, Tuple

# 添加 proto 路径
sys.path.insert(0, 'src/robot_proto/python')

try:
    from robot.v1 import system_pb2, system_pb2_grpc
    from robot.v1 import control_pb2, control_pb2_grpc
    from robot.v1 import data_pb2, data_pb2_grpc
except ImportError:
    print("❌ 无法导入 proto 存根")
    print("   提示: 请先生成 Python proto 代码")
    print("   运行: bash scripts/proto_gen.sh")
    sys.exit(1)


class Colors:
    """终端颜色"""
    GREEN = '\033[0;32m'
    RED = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'


class GrpcTester:
    """gRPC 测试器"""

    def __init__(self, host='localhost:50051'):
        self.host = host
        self.channel = None
        self.system_client = None
        self.control_client = None
        self.data_client = None

    def connect(self) -> bool:
        """连接到 gRPC 服务器"""
        try:
            self.channel = grpc.insecure_channel(self.host)
            # 等待连接就绪（最多 5 秒）
            grpc.channel_ready_future(self.channel).result(timeout=5)

            self.system_client = system_pb2_grpc.SystemServiceStub(self.channel)
            self.control_client = control_pb2_grpc.ControlServiceStub(self.channel)
            self.data_client = data_pb2_grpc.DataServiceStub(self.channel)

            print(f"{Colors.GREEN}✓{Colors.NC} 已连接到 {self.host}")
            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.channel:
            self.channel.close()

    def test_get_system_info(self) -> bool:
        """测试 GetSystemInfo RPC"""
        try:
            request = system_pb2.GetSystemInfoRequest()
            response = self.system_client.GetSystemInfo(request, timeout=5.0)

            assert response.version != "", "版本号不能为空"
            assert response.uptime_seconds >= 0, "运行时间不能为负"

            print(f"{Colors.GREEN}✓{Colors.NC} GetSystemInfo 测试通过")
            print(f"  版本: {response.version}")
            print(f"  运行时间: {response.uptime_seconds}s")
            print(f"  CPU 使用率: {response.cpu_percent:.1f}%")
            print(f"  内存使用率: {response.memory_percent:.1f}%")
            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} GetSystemInfo 测试失败: {e}")
            return False

    def test_acquire_release_lease(self) -> bool:
        """测试 AcquireLease 和 ReleaseLease RPC"""
        try:
            # 获取租约
            acquire_req = system_pb2.AcquireLeaseRequest(
                client_id="test_client",
                duration_seconds=60
            )
            acquire_resp = self.system_client.AcquireLease(acquire_req, timeout=5.0)

            assert acquire_resp.error_code == system_pb2.ERROR_CODE_SUCCESS, \
                f"获取租约失败: {acquire_resp.error_code}"

            lease_id = acquire_resp.lease_id
            print(f"{Colors.GREEN}✓{Colors.NC} AcquireLease 测试通过")
            print(f"  租约 ID: {lease_id}")

            # 释放租约
            release_req = system_pb2.ReleaseLeaseRequest(lease_id=lease_id)
            release_resp = self.system_client.ReleaseLease(release_req, timeout=5.0)

            assert release_resp.error_code == system_pb2.ERROR_CODE_SUCCESS, \
                f"释放租约失败: {release_resp.error_code}"

            print(f"{Colors.GREEN}✓{Colors.NC} ReleaseLease 测试通过")
            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} Lease 测试失败: {e}")
            return False

    def test_get_mode(self) -> bool:
        """测试 GetMode RPC"""
        try:
            request = control_pb2.GetModeRequest()
            response = self.control_client.GetMode(request, timeout=5.0)

            assert response.mode != "", "模式不能为空"

            print(f"{Colors.GREEN}✓{Colors.NC} GetMode 测试通过")
            print(f"  当前模式: {response.mode}")
            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} GetMode 测试失败: {e}")
            return False

    def test_list_maps(self) -> bool:
        """测试 ListMaps RPC"""
        try:
            request = system_pb2.ListMapsRequest()
            response = self.system_client.ListMaps(request, timeout=5.0)

            print(f"{Colors.GREEN}✓{Colors.NC} ListMaps 测试通过")
            print(f"  地图数量: {len(response.maps)}")

            for i, map_info in enumerate(response.maps[:3]):
                print(f"    {i+1}. {map_info.name} ({map_info.size_bytes} bytes)")

            if len(response.maps) > 3:
                print(f"    ... 还有 {len(response.maps) - 3} 个地图")

            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} ListMaps 测试失败: {e}")
            return False

    def test_get_installed_versions(self) -> bool:
        """测试 GetInstalledVersions RPC"""
        try:
            request = data_pb2.GetInstalledVersionsRequest()
            response = self.data_client.GetInstalledVersions(request, timeout=5.0)

            print(f"{Colors.GREEN}✓{Colors.NC} GetInstalledVersions 测试通过")
            print(f"  已安装版本数: {len(response.versions)}")

            for version in response.versions[:3]:
                print(f"    - {version.name}: {version.version}")

            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} GetInstalledVersions 测试失败: {e}")
            return False

    def test_performance(self) -> bool:
        """测试性能（100 次 GetSystemInfo 调用）"""
        try:
            iterations = 100
            print(f"执行性能测试 ({iterations} 次调用)...")

            request = system_pb2.GetSystemInfoRequest()

            start = time.time()
            success_count = 0

            for _ in range(iterations):
                try:
                    self.system_client.GetSystemInfo(request, timeout=1.0)
                    success_count += 1
                except:
                    pass

            end = time.time()
            duration = end - start

            qps = success_count / duration
            avg_latency = (duration / success_count) * 1000

            print(f"{Colors.GREEN}✓{Colors.NC} 性能测试完成")
            print(f"  成功: {success_count}/{iterations}")
            print(f"  总耗时: {duration:.2f}s")
            print(f"  QPS: {qps:.2f} req/s")
            print(f"  平均延迟: {avg_latency:.2f} ms")

            return success_count >= iterations * 0.95  # 95% 成功率
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 性能测试失败: {e}")
            return False

    def run_all_tests(self) -> Tuple[int, int]:
        """运行所有测试"""
        tests = [
            ("GetSystemInfo", self.test_get_system_info),
            ("AcquireLease/ReleaseLease", self.test_acquire_release_lease),
            ("GetMode", self.test_get_mode),
            ("ListMaps", self.test_list_maps),
            ("GetInstalledVersions", self.test_get_installed_versions),
            ("性能测试", self.test_performance),
        ]

        passed = 0
        failed = 0

        print("=" * 60)
        print("  gRPC 接口集成测试")
        print("=" * 60)
        print()

        for name, test_func in tests:
            print(f"{Colors.BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.NC}")
            print(f"{Colors.BLUE}测试: {name}{Colors.NC}")
            print(f"{Colors.BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.NC}")

            if test_func():
                passed += 1
            else:
                failed += 1

            print()

        return passed, failed


def main():
    """主函数"""
    print("等待 gRPC Gateway 启动...")
    time.sleep(2)

    tester = GrpcTester()

    # 连接到服务器
    if not tester.connect():
        print()
        print(f"{Colors.RED}❌ 无法连接到 gRPC Gateway{Colors.NC}")
        print()
        print("提示:")
        print("  1. 确保系统正在运行: make navigation")
        print("  2. 检查端口是否正确: 50051")
        print("  3. 检查防火墙设置")
        sys.exit(1)

    print()

    # 运行测试
    passed, failed = tester.run_all_tests()

    # 断开连接
    tester.disconnect()

    # 输出结果
    print("=" * 60)
    print(f"  测试结果: {passed} 通过, {failed} 失败")
    print("=" * 60)

    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
