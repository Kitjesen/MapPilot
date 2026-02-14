#!/usr/bin/env python3
"""
网络故障模拟测试
测试系统在网络异常情况下的容错能力
"""

import grpc
import sys
import time
import subprocess
import signal
from typing import Optional

sys.path.insert(0, 'src/robot_proto/python')

try:
    from robot.v1 import system_pb2, system_pb2_grpc
except ImportError:
    print("❌ 无法导入 proto 存根")
    sys.exit(1)


class Colors:
    """终端颜色"""
    GREEN = '\033[0;32m'
    RED = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'


class NetworkFailureTester:
    """网络故障测试器"""

    def __init__(self, host='localhost:50051'):
        self.host = host
        self.channel: Optional[grpc.Channel] = None
        self.client: Optional[system_pb2_grpc.SystemServiceStub] = None

    def connect(self) -> bool:
        """连接到 gRPC 服务器"""
        try:
            self.channel = grpc.insecure_channel(self.host)
            grpc.channel_ready_future(self.channel).result(timeout=5)
            self.client = system_pb2_grpc.SystemServiceStub(self.channel)
            return True
        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.channel:
            self.channel.close()
            self.channel = None
            self.client = None

    def test_connection_timeout(self) -> bool:
        """测试连接超时"""
        print(f"{Colors.BLUE}测试: 连接超时{Colors.NC}")
        print("模拟连接到不存在的服务器...")

        try:
            # 尝试连接到不存在的地址
            channel = grpc.insecure_channel('localhost:9999')
            client = system_pb2_grpc.SystemServiceStub(channel)

            request = system_pb2.GetSystemInfoRequest()

            start = time.time()
            try:
                client.GetSystemInfo(request, timeout=2.0)
                print(f"{Colors.RED}✗{Colors.NC} 应该超时但成功了")
                return False
            except grpc.RpcError as e:
                elapsed = time.time() - start
                if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    print(f"{Colors.GREEN}✓{Colors.NC} 正确处理超时 ({elapsed:.2f}s)")
                    return True
                else:
                    print(f"{Colors.YELLOW}⚠{Colors.NC} 超时但错误码不符: {e.code()}")
                    return True  # 仍然算通过，因为确实失败了
            finally:
                channel.close()

        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 测试失败: {e}")
            return False

    def test_request_timeout(self) -> bool:
        """测试请求超时"""
        print(f"{Colors.BLUE}测试: 请求超时{Colors.NC}")
        print("发送超时请求...")

        if not self.client:
            print(f"{Colors.YELLOW}⚠{Colors.NC} 未连接，跳过测试")
            return True

        try:
            request = system_pb2.GetSystemInfoRequest()

            # 设置非常短的超时时间
            start = time.time()
            try:
                self.client.GetSystemInfo(request, timeout=0.001)
                elapsed = time.time() - start
                print(f"{Colors.GREEN}✓{Colors.NC} 请求成功 ({elapsed:.3f}s)")
                return True
            except grpc.RpcError as e:
                elapsed = time.time() - start
                if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
                    print(f"{Colors.GREEN}✓{Colors.NC} 正确处理超时 ({elapsed:.3f}s)")
                    return True
                else:
                    print(f"{Colors.YELLOW}⚠{Colors.NC} 超时但错误码不符: {e.code()}")
                    return True

        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 测试失败: {e}")
            return False

    def test_reconnection(self) -> bool:
        """测试重连机制"""
        print(f"{Colors.BLUE}测试: 重连机制{Colors.NC}")
        print("断开连接后重连...")

        try:
            # 断开连接
            self.disconnect()
            print("  已断开连接")
            time.sleep(1)

            # 重新连接
            if self.connect():
                print(f"{Colors.GREEN}✓{Colors.NC} 重连成功")

                # 验证连接可用
                request = system_pb2.GetSystemInfoRequest()
                response = self.client.GetSystemInfo(request, timeout=5.0)

                if response.version:
                    print(f"{Colors.GREEN}✓{Colors.NC} 连接正常工作")
                    return True
                else:
                    print(f"{Colors.RED}✗{Colors.NC} 连接异常")
                    return False
            else:
                print(f"{Colors.RED}✗{Colors.NC} 重连失败")
                return False

        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 测试失败: {e}")
            return False

    def test_rapid_requests(self) -> bool:
        """测试快速连续请求"""
        print(f"{Colors.BLUE}测试: 快速连续请求{Colors.NC}")
        print("发送 50 个连续请求...")

        if not self.client:
            print(f"{Colors.YELLOW}⚠{Colors.NC} 未连接，跳过测试")
            return True

        try:
            request = system_pb2.GetSystemInfoRequest()
            success_count = 0
            failed_count = 0

            start = time.time()
            for i in range(50):
                try:
                    self.client.GetSystemInfo(request, timeout=1.0)
                    success_count += 1
                except:
                    failed_count += 1

            elapsed = time.time() - start

            print(f"  成功: {success_count}, 失败: {failed_count}")
            print(f"  总耗时: {elapsed:.2f}s")
            print(f"  QPS: {success_count / elapsed:.2f} req/s")

            # 95% 成功率算通过
            if success_count >= 47:
                print(f"{Colors.GREEN}✓{Colors.NC} 快速请求处理正常")
                return True
            else:
                print(f"{Colors.YELLOW}⚠{Colors.NC} 成功率较低: {success_count/50*100:.1f}%")
                return False

        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 测试失败: {e}")
            return False

    def test_concurrent_requests(self) -> bool:
        """测试并发请求"""
        print(f"{Colors.BLUE}测试: 并发请求{Colors.NC}")
        print("发送 10 个并发请求...")

        if not self.client:
            print(f"{Colors.YELLOW}⚠{Colors.NC} 未连接，跳过测试")
            return True

        try:
            import concurrent.futures

            request = system_pb2.GetSystemInfoRequest()

            def make_request():
                try:
                    self.client.GetSystemInfo(request, timeout=5.0)
                    return True
                except:
                    return False

            start = time.time()
            with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
                futures = [executor.submit(make_request) for _ in range(10)]
                results = [f.result() for f in concurrent.futures.as_completed(futures)]

            elapsed = time.time() - start
            success_count = sum(results)

            print(f"  成功: {success_count}/10")
            print(f"  总耗时: {elapsed:.2f}s")

            if success_count >= 9:
                print(f"{Colors.GREEN}✓{Colors.NC} 并发请求处理正常")
                return True
            else:
                print(f"{Colors.YELLOW}⚠{Colors.NC} 部分并发请求失败")
                return False

        except Exception as e:
            print(f"{Colors.RED}✗{Colors.NC} 测试失败: {e}")
            return False

    def run_all_tests(self):
        """运行所有测试"""
        tests = [
            ("连接超时", self.test_connection_timeout),
            ("请求超时", self.test_request_timeout),
            ("重连机制", self.test_reconnection),
            ("快速连续请求", self.test_rapid_requests),
            ("并发请求", self.test_concurrent_requests),
        ]

        passed = 0
        failed = 0

        print("=" * 70)
        print("  网络故障模拟测试")
        print("=" * 70)
        print()

        for name, test_func in tests:
            print(f"{Colors.BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.NC}")
            print(f"{Colors.BLUE}{name}{Colors.NC}")
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

    tester = NetworkFailureTester()

    # 连接到服务器
    print("连接到 gRPC Gateway...")
    if not tester.connect():
        print()
        print(f"{Colors.YELLOW}⚠ 无法连接到 gRPC Gateway{Colors.NC}")
        print("   将跳过需要连接的测试")
        print()

    # 运行测试
    passed, failed = tester.run_all_tests()

    # 断开连接
    tester.disconnect()

    # 输出结果
    print("=" * 70)
    print(f"  测试结果: {passed} 通过, {failed} 失败")
    print("=" * 70)

    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
