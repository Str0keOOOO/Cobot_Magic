from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tiptop_client.core.config import load_config
from tiptop_client.core.errors import ConfigurationError


class ConfigTest(unittest.TestCase):
    def test_defaults_and_override(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bridge.yaml"
            path.write_text("robot:\n  servo_rate_hz: 123.0\n", encoding="utf-8")
            config = load_config(path)
        self.assertEqual(config["robot"]["servo_rate_hz"], 123.0)
        self.assertEqual(config["robot"]["arm_dof"], 6)

    def test_invalid_top_level_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bridge.yaml"
            path.write_text("- not-a-mapping\n", encoding="utf-8")
            with self.assertRaises(ConfigurationError):
                load_config(path)
