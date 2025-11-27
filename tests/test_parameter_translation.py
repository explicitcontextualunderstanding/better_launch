
import pytest
import json
import sys
from unittest.mock import MagicMock

# Mock ROS dependencies BEFORE importing better_launch
def mock_package(name):
    m = MagicMock()
    m.__path__ = []
    sys.modules[name] = m
    return m

mock_rclpy = mock_package("rclpy")
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.logging"] = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["rclpy.task"] = MagicMock()
sys.modules["rclpy.executors"] = MagicMock()
sys.modules["rclpy.parameter"] = MagicMock()

mock_ament = mock_package("ament_index_python")
sys.modules["ament_index_python.packages"] = MagicMock()

mock_lifecycle_msgs = mock_package("lifecycle_msgs")
sys.modules["lifecycle_msgs.msg"] = MagicMock()
sys.modules["lifecycle_msgs.srv"] = MagicMock()

mock_launch = mock_package("launch")
sys.modules["launch.actions"] = MagicMock()
sys.modules["launch.launch_description_sources"] = MagicMock()

from better_launch.launcher import BetterLaunch

class TestParameterTranslation:
    @pytest.fixture
    def bl(self):
        # Create a mock BetterLaunch instance to access the method
        # We avoid full instantiation to avoid side effects (ROS init, singleton, etc.)
        bl = MagicMock(spec=BetterLaunch)
        # Bind the method to the mock instance
        bl._to_ros2_yaml = BetterLaunch._to_ros2_yaml.__get__(bl, BetterLaunch)
        return bl

    @pytest.mark.parametrize("input_val, expected_output", [
        (True, "true"),
        (False, "false"),
        (42, "42"),
        (3.14, "3.14"),
        ("hello", "hello"),
        ("true", "true"), # String "true" should stay "true"
        ([1, 2, 3], "[1, 2, 3]"),
        ({"a": 1, "b": 2}, '{"a": 1, "b": 2}'),
        (None, ""),
    ])
    def test_to_ros2_yaml(self, bl, input_val, expected_output):
        """Test that _to_ros2_yaml correctly converts Python types to ROS2-compatible YAML strings."""
        assert bl._to_ros2_yaml(input_val) == expected_output

    def test_complex_nested_structure(self, bl):
        """Test nested structures."""
        data = {"list": [1, 2], "bool": True, "nested": {"x": "y"}}
        # JSON serialization order might vary, but for this simple case it's usually stable.
        # However, to be safe, we can parse the output back and compare.
        output = bl._to_ros2_yaml(data)
        assert json.loads(output) == data

    def test_string_quoting(self, bl):
        """Test that strings are NOT quoted (passed as-is)."""
        # ROS2 launch arguments are typically just strings.
        # If we pass "foo", it receives "foo".
        # If we pass '"foo"', it receives '"foo"'.
        assert bl._to_ros2_yaml("foo") == "foo"
        assert bl._to_ros2_yaml("foo bar") == "foo bar"
