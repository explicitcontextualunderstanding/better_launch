import timeit
import json
import sys
from unittest.mock import MagicMock

# Mock ROS dependencies
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["launch"] = MagicMock()
sys.modules["launch.actions"] = MagicMock()
sys.modules["launch.launch_description_sources"] = MagicMock()
sys.modules["ament_index_python"] = MagicMock()
sys.modules["ament_index_python.packages"] = MagicMock()

from better_launch.launcher import BetterLaunch

def benchmark():
    bl = BetterLaunch()
    
    # Test cases
    cases = {
        "bool_true": True,
        "bool_false": False,
        "int_small": 42,
        "float_pi": 3.14159,
        "str_short": "hello",
        "list_small": [1, 2, 3],
        "dict_small": {"a": 1, "b": 2},
        "float_nan": float('nan'),
    }
    
    # Mock Substitution
    class MockSubstitution:
        def perform(self, context): return "sub"
    cases["substitution"] = MockSubstitution()

    iterations = 100000
    
    print(f"Running benchmark with {iterations} iterations per case...\n")
    print(f"{'Type':<20} | {'Baseline (str)':<15} | {'Translator':<15} | {'Factor':<10}")
    print("-" * 70)

    for name, val in cases.items():
        # Baseline: str()
        # Note: str() is not correct for all types (e.g. bool) but serves as a speed baseline
        t_base = timeit.timeit(lambda: str(val), number=iterations)
        
        # Translator
        t_trans = timeit.timeit(lambda: bl._to_ros2_yaml(val), number=iterations)
        
        factor = t_trans / t_base if t_base > 0 else 0
        
        print(f"{name:<20} | {t_base:.5f}s        | {t_trans:.5f}s        | {factor:.2f}x")

if __name__ == "__main__":
    benchmark()
