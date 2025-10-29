"""Additional convenience methods that aren't general enough to be added to the main namespace."""

__all__ = [
    "rviz",
    "read_robot_description",
    "joint_state_publisher",
    "robot_state_publisher",
    "static_transform_publisher",
]


from typing import Sequence, Any
import subprocess
import json

from better_launch import BetterLaunch
from better_launch.elements import Node


def rviz(
    package: str = None,
    configfile: str = None,
    subdir: str = None,
    *,
    extra_args: list[str] = None,
) -> Node:
    """Runs RViz2 with the given config file and optional warning level suppression.

    Parameters
    ----------
    package : str, optional
        Path to locate the config file in (if one is specified).
    config_file : str, optional
        Path to the RViz2 configuration file which will be resolved by :py:meth:`BetterLaunch.find`. Otherwise RViz2 will run with the default config.
    subdir : str, optional
        A path fragment the config file must be located in.
    extra_args : list[str], optional
        Additional args to pass to the RViz2 executable.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    args = []
    if configfile:
        configfile = bl.find(package, configfile, subdir)
        args += ["-d", configfile]

    if extra_args:
        args.extend(extra_args)

    # rviz2 doesn't support the --log-level argument nodes usually accept
    return bl.node(
        "rviz2", "rviz2", "rviz2", anonymous=True, cmd_args=args, log_level=None
    )


def read_robot_description(
    package: str,
    description_file: str,
    subdir: str = None,
    *,
    xacro_args: list[str] = None,
) -> str | None:
    """Returns the contents of a robot description after a potential xacro parse.

    The file is resolved using :py:meth:`BetterLaunch.find`. If the description file ends with `.urdf` and `xacro_args` is not provided, it reads the URDF file directly. Otherwise it runs `xacro` to generate the URDF from a `.xacro` file.

    Parameters
    ----------
    package : str
        The package where the robot description file is located. May be `None` to use this launch file's package (see :py:meth:`BetterLaunch.find`).
    description_file : str
        The name of the robot description file (URDF or XACRO).
    subdir : str, optional
        A path fragment the description file must be located in.
    xacro_args : list of str, optional
        Additional arguments to pass to `xacro` when processing `.xacro` files.

    Returns
    -------
    str | None
        The parsed URDF XML as a string if successful, `None` otherwise.

    Raises
    ------
    ValueError
        If the xacro command encountered an error while processing the file.
    """
    bl = BetterLaunch.instance()

    filepath = bl.find(package, description_file, subdir)

    if not filepath.endswith("xacro") and xacro_args is None:
        with open(filepath) as f:
            return f.read()

    args = [filepath]
    if xacro_args:
        args.extend(xacro_args)

    try:
        return bl.exec(["xacro", *args])
    except subprocess.CalledProcessError as e:
        raise ValueError(f"Xacro failed ({e.returncode}): {e.output}") from e


def joint_state_publisher(
    use_gui: bool = False, node_name: str = None, **kwargs
) -> Node:
    """Starts a `joint_state_publisher` or `joint_state_publisher_gui` node.

    Parameters
    ----------
    use_gui : bool, optional
        Whether to use the GUI version of the `joint_state_publisher`.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments to pass to the node (e.g. name, remaps, params, etc.). See :py:meth:`BetterLaunch.node`.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    kwargs.setdefault("anonymous", True)

    if use_gui:
        return bl.node(
            "joint_state_publisher_gui",
            "joint_state_publisher_gui",
            node_name or "joint_state_publisher_gui",
            **kwargs,
        )
    else:
        return bl.node(
            "joint_state_publisher",
            "joint_state_publisher",
            node_name or "joint_state_publisher",
            **kwargs,
        )


def robot_state_publisher(
    package: str,
    description_file: str,
    subdir: str = None,
    *,
    xacro_args: list[str] = None,
    node_name: str = None,
    **kwargs,
) -> Node:
    """Start a Robot State Publisher node using the given URDF/Xacro file. The file is resolved using :py:meth:`BetterLaunch.find`.

    Parameters
    ----------
    package : str
        The name of the package containing the robot description file. May be `None` to use this launch file's package (see :py:meth:`BetterLaunch.find`).
    description_file : str
        The name of the robot description for the robot. Typically a .sdf, .urdf or .xacro file.
    subdir : str, optional
        A path fragment the description file must be located in.
    xacro_args : list of str, optional
        Additional arguments to pass to the Xacro processor when processing `.xacro` files.
    node_name : str, optional
        The name of the node. If not provided the name of the executable will be used. Will be anonymized unless `anonymous=False` is passed.
    **kwargs : dict, optional
        Additional arguments for the node, such as remappings or parameters.

    Returns
    -------
    Node
        The spawned node instance.
    """
    bl = BetterLaunch.instance()

    robot_description = read_robot_description(
        package,
        description_file,
        subdir,
        xacro_args=xacro_args,
    )

    kwargs.setdefault("anonymous", True)
    params = kwargs.pop("params", {})
    params["robot_description"] = robot_description

    return bl.node(
        "robot_state_publisher",
        "robot_state_publisher",
        node_name,
        params=params,
        **kwargs,
    )


def static_transform_publisher(
    parent_frame: str,
    child_frame: str,
    pos: Sequence[float] = None,
    rot: Sequence[float] = None,
) -> Node:
    """Publish a static transform between two frames.

    Parameters
    ----------
    parent_frame : str
        The parent or source frame.
    child_frame : str
        The child or target frame.
    pos : Sequence[float], optional
        The xyz-translation from parent to child frame. You may also pass a sequence of 6 or 7 floats in order to specify a full pose. In this case, `rot` will be ignored.
    rot : Sequence[float], optional
        The rotation between the parent and child frame. If length is 3, the values are interpreted as roll-pitch-yaw euler angles. If length is 4, an xyzw-quaternion is assumed.

    Returns
    -------
    Node
        The node running the publisher process.

    Raises
    ------
    ValueError
        If pos or rot have the wrong length.
    """
    args = ["--frame-id", parent_frame, "--child-frame-id", child_frame]

    # Position may also hold the pose
    if pos is not None:
        args.extend(["--x", pos[0], "--y", pos[1], "--z", pos[2]])
        if len(pos) == 3:
            pass
        elif len(pos) == 4 and pos[3] == 1.0:
            # Homogenous translation vector, it's fine
            pass
        elif len(pos) in (6, 7):
            rot = pos[3:]
        else:
            raise ValueError("Position has dubious length %d", len(pos))

    if rot is not None:
        if len(rot) == 3:
            args.extend(["--roll", rot[0], "--pitch", rot[1], "--yaw", rot[2]])
        elif len(rot) == 4:
            args.extend(
                ["--qx", rot[0], "--qy", rot[1], "--qz", rot[2], "--qw", rot[3]]
            )
        else:
            raise ValueError("Rotation has dubious length %d", len(rot))

    bl = BetterLaunch.instance()
    return bl.node(
        "tf2_ros",
        "static_transform_publisher",
        "gazebo_world_tf",
        cmd_args=args,
        log_level=None,
    )


def spawn_controller_manager(
    params: str | dict[str, Any] = None,
    *,
    remaps: dict[str, str] = None,
    cmd_args: list[str] = None,
    robot_description: str = None,
    name: str = "controller_manager",
) -> Node:
    """Spawn a new controller manager.

    Parameters
    ----------
    manager_config : str | dict[str, Any], optional
        The controller manager config to use (typically named `controller.yaml`). If a string is passed it is considered as a path and loaded via :py:meth:`BetterLaunch.load_params`.
    remaps : dict[str, str], optional
        Topic remaps for the controller manager, e.g. for the `~/robot_description` topic it usually subscribes to.
    cmd_args: list[str], optional
        Additional CLI arguments to pass to the spawner command (e.g. `--load-only`).
    robot_description : str, optional
        The contents of a robot description. This is not recommended anymore, but see :py:meth:`read_robot_description` if you are interested nonetheless. If not provided, the description will be read from the `~/robot_description` topic.
    name : str, optional
        The name the controller manager node should use. The rename is qualified and so won't affect controllers spawned later (see `this document <https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#using-the-controller-manager-in-a-process>`_ for details). Note however that many nodes and CLI programs (e.g. `ros2 control`) expect the manager to be named `controller_manager` and won't work properly otherwise.

    Returns
    -------
    Node
        The node running the controller manager process.
    """
    bl = BetterLaunch.instance()

    # In ROS2 there isn't a central parameter server anymore. However, each ROS2 process stores
    # context object with all ros args passed to it. If the process creates new nodes their
    # startup arguments are populated from this context object. This way the controller manager
    # can store parameters for other controllers even if they are only spawned later.
    # See this very helpful summary for details:
    # https://github.com/ros-controls/ros2_control/issues/335

    if params is None:
        params = {}
    elif isinstance(params, str):
        # In theory you could pass the config and then change it before a controller is loaded, 
        # but that seems debatable at best. If you truly want this, either pass the file path as
        # a cmd arg, or keep the manager and controller configs separate and pass the later to 
        # spawn_controller below.
        # process_args.extend(["--param-file", params])
        params = bl.load_params(None, params, matching_only=False)

    if robot_description:
        params["robot_description"] = robot_description

    return bl.node(
        package="controller_manager",
        executable="ros2_control_node",
        name=name,
        remaps=remaps,
        params=params,
        cmd_args=cmd_args,
        # Prevent renaming nodes spawned by the manager
        # See https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#using-the-controller-manager-in-a-process
        remap_qualifier="controller_manager",
    )


def spawn_controller(
    controller: str,
    params: str | list[str] | dict[str, Any] = None,
    *,
    remaps: dict[str, str] = None,
    cmd_args: list[str] = None,
    manager: str = "controller_manager",
) -> None:  # TODO find a way to return an interactable object
    """Spawn the specified controller.

    Note that right now there is no way to directly interact with the loaded controller's node due to the limited API ROS2 provides. I will find a way...

    Parameters
    ----------
    controller : str
        The controller to spwawn.
    params : str | list[str] | dict[str, Any], optional
        Additional parameters for the controller node. Can be a path to a ROS2 config, a list of paths, or a dict with the actual key-value pairs.
    remaps : dict[str, str], optional
        Additional remaps specific to the controller. These will be qualified with the controller's name to avoid conflicts.
    cmd_args: list[str], optional
        Additional CLI arguments to pass to the spawner command (e.g. `--load-only`).
    manager : str, optional
        The name of the controller_manager node.
    """
    bl = BetterLaunch.instance()
    process_args = [controller, "--controller-manager", manager]

    if cmd_args:
        process_args.extend(cmd_args)

    if isinstance(params, str):
        params = bl.load_params(None, params)

    if params:
        if isinstance(params, dict):
            if bl.ros_distro()[0].lower() < "j":
                raise ValueError(
                    "Passing controller params directly is only supported in Jazzy and newer"
                )
            
            manager_node = bl.query_node(manager, include_foreign=True)

            if not manager_node:
                raise ValueError(f'Could not find controller manager "{manager}"')

            # In theory we could pass --controller-ros-args to the spawner and let the spawner 
            # handle these, but it unfortunately does some very naive string splitting which 
            # messes up more complex arguments containing e.g. lists. 
            manager_node.set_live_params(
                {
                    f"{controller}.node_options_args": [
                        f"{key}:={json.dumps(val)}"
                        for key, val in params.items()
                    ]
                }
            )
        elif isinstance(params, str):
            # Usually we'd load the parameters here and pass them to the controller manager, 
            # but this functionality only exists from jazzy onwards
            process_args.extend(["--param-file", params])
        elif isinstance(params, list):
            for path in params:
                process_args.extend(["--param-file", path])
        else:
            raise ValueError(f"Controller params of type {type(params)} not supported")

    if remaps:
        if bl.ros_distro()[0].lower() < "j":
            raise ValueError(
                "Passing controller params directly is only supported in Jazzy and newer"
            )

        for key, value in remaps.items():
            # Qualify remaps to avoid accidental remaps for other controllers 
            process_args.extend(
                ["--controller-ros-args", f"-r {controller}:{key}:={value}"]
            )

    # This is NOT a node! Could also use the spawner python implementation directly, but that
    # would just introduce another dependency with little benefit.
    spawner = bl.find("controller_manager", "controller_manager/spawner")
    bl.exec(["python3", spawner] + process_args)
