"""
Output controllers: everything that consumes the per-cycle localization results.
Each controller module is imported only when it is selected in OUTPUTS, so optional
dependencies (e.g. ROS 2) are never imported unless they are used.
"""
import importlib
from outputs.base import OutputController

# Output name -> (module, class). Modules are imported lazily in make_output_controllers().
_CONTROLLERS = {
    "console": ("outputs.console", "ConsoleOutputController"),
    "csv":     ("outputs.csv_output", "CsvOutputController"),
    "dynamic": ("outputs.dynamic", "DynamicDisplayController"),
    "ros2":    ("outputs.ros2_output", "Ros2OutputController"),
}
ROS2_IMPORT_ERROR = ("ROS 2 output selected but rclpy not importable. Did you `source /opt/ros/jazzy/setup.bash` "
                     "and create the venv with --system-site-packages?")


def make_output_controllers(outputs, tag_id) -> list[OutputController]:
    """
    Builds one output controller per name in outputs (see OUTPUTS in config.py).
    Raises ValueError on an unknown name.
    """
    controllers = []
    for name in outputs:
        if name not in _CONTROLLERS:
            raise ValueError(f"Unknown output '{name}'. Available outputs: {', '.join(_CONTROLLERS)}")
        module_name, class_name = _CONTROLLERS[name]
        try:
            module = importlib.import_module(module_name)
        except ImportError as e:
            if name == "ros2":
                raise ImportError(ROS2_IMPORT_ERROR) from e
            raise
        controllers.append(getattr(module, class_name)(tag_id))
    return controllers
