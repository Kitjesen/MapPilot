"""Test LingTu SDK client with mocked HTTP."""
import json, unittest
from unittest.mock import patch
from lingtu_sdk import LingTuClient

class TestLingTuClient(unittest.TestCase):
    def setUp(self): self.robot = LingTuClient()

    @patch('urllib.request.urlopen')
    def test_go(self, mock_urlopen):
        mock_urlopen.return_value.__enter__.return_value.read.return_value = b'{"ok": true}'
        r = self.robot.go(10, 5)
        self.assertEqual(r, {"ok": True})

    @patch('urllib.request.urlopen')
    def test_state(self, mock_urlopen):
        mock_urlopen.return_value.__enter__.return_value.read.return_value = b'{"odometry": {"x": 1.0}}'
        self.assertEqual(self.robot.position(), {"x": 1.0, "y": 0, "z": 0, "yaw": 0})

    @patch('urllib.request.urlopen')
    def test_context_manager(self, mock_urlopen):
        with LingTuClient() as r:
            self.assertIsNotNone(r)

if __name__ == "__main__":
    unittest.main()
