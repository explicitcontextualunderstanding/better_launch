
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

    def test_non_serializable(self, bl):
        """Test that non-serializable types raise ValueError."""
        class CustomObj:
            pass
        
        with pytest.raises(ValueError, match="Failed to serialize launch argument"):
            bl._to_ros2_yaml(CustomObj())

    @pytest.mark.parametrize("input_val, expected_output", [
        (True, "true"),
        ({"a": 1}, '{"a": 1}'),
    ])
    def test_include_calls_translation(self, input_val, expected_output):
        """Verify that _include_ros2_launchfile calls _to_ros2_yaml."""
        # We need a real BetterLaunch instance (or close to it) to test the call chain
        # but we can mock the internal methods
        bl = BetterLaunch()
        
        # Mock _to_ros2_yaml to verify it's called
        # We wrap the real method to check return values too if needed, 
        # but here we just want to ensure it's invoked.
        # However, since we want to test the integration, let's mock ros2_actions
        # and see what arguments it receives.
        
        bl.ros2_actions = MagicMock()
        bl.find = MagicMock(return_value="/dummy/path.launch.py")
        
        # Call _include_ros2_launchfile
        bl._include_ros2_launchfile("/dummy/path.launch.py", my_arg=input_val)
        
        # Verify ros2_actions was called
        assert bl.ros2_actions.called
        
        # Inspect the IncludeLaunchDescription object passed to ros2_actions
        ros2_include = bl.ros2_actions.call_args[0][0]
        from launch.actions import IncludeLaunchDescription
        # Since we mocked launch.actions, IncludeLaunchDescription is a MagicMock
        # We can check if it was created from that mock
        # assert isinstance(ros2_include, IncludeLaunchDescription) or ros2_include.__class__.__name__ == 'MagicMock'
        # Simply checking it's a mock is enough for this unit test
        assert hasattr(ros2_include, 'launch_arguments')
        
        # Check launch_arguments
        # launch_arguments is a list of tuples: [('my_arg', 'expected_output')]
        launch_args = dict(ros2_include.launch_arguments)
        assert launch_args['my_arg'] == expected_output
