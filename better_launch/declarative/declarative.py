from typing import Any, Literal
import inspect
import contextlib
import logging

from better_launch import BetterLaunch
from better_launch.wrapper import _exec_launch_func
from better_launch.utils.better_logging import Colormode
from better_launch.utils.click import LaunchArg

from .toml_parser import load as load_toml
from .substitutions import apply_substitutions


toml_format_version = 1


def _execute_toml(toml: dict[str, Any], eval_mode: Literal["full", "literal", "none"]) -> dict[str, Any]:
    """Execute each call table and apply substitutions."""
    if BetterLaunch.instance():
        raise RuntimeError("BetterLaunch has already been initialized")

    # Initialize the launcher instance
    bl = BetterLaunch()
    valid_funcs = set(f for f in BetterLaunch.__dict__ if not f.startswith("_"))
    results = dict(bl.launch_args)

    def exec_request(key: str, req: dict) -> Any:
        if "func" not in req:
            return

        for attr, val in req.items():
            if isinstance(val, str):
                req[attr] = apply_substitutions(val, None, results, eval_type=eval_mode)

        if not req.pop("if", True):
            return

        if req.pop("unless", False):
            return

        # If not specified assume we're creating a node
        func_name = req.pop("func")
        if func_name not in valid_funcs:
            raise KeyError(f"func='{func_name}' is not a valid request")

        func = getattr(bl, func_name)
        func_sig = inspect.signature(func)

        # Where accepted the table key can be used as the name (e.g. of a node)
        if "name" in func_sig.parameters and "name" not in req:
            req["name"] = key

        # Call the function and store the result
        children = None
        if "children" not in func_sig.parameters:
            children = req.pop("children", None)

        res = func(**req)
        results[key] = res

        if children:
            if not isinstance(res, contextlib.AbstractContextManager):
                logging.getLogger().warning(
                    f"{req} has 'children' key, but did not result in a context object - children will be ignored"
                )
                return

            with res:
                # Must be a proper TOML subtable, we don't accept arrays of tables here
                if not isinstance(children, dict):
                    raise ValueError(f"Children of {key} must be specified as a dict")

                for subkey, child in children.items():
                    exec_request(subkey, child)

    for key, val in toml.items():
        if isinstance(val, dict):
            exec_request(key, val)
        else:
            results[key] = val

    return results


def _get_toml_launch_args(toml: dict) -> list[LaunchArg]:
    args = []

    for key, val in toml.items():
        if key.startswith("_"):
            continue

        if isinstance(val, dict) and "func" in val:
            continue

        if isinstance(val, type):
            # no default value
            ptype = val
            default = None
        else:
            ptype = type(val)
            default = val

        description = toml.get(f"__comment_{key}__")

        args.append(
            LaunchArg(
                key,
                ptype,
                default,
                description,
            )
        )

    return args


def launch_toml(path: str,) -> None:
    """Execute a TOML better_launch launchfile.

    In better_launch TOML launch files, most tables will be `call tables`. A call table is a dict that has a `func` key referring to one of the public :py:class:`BetterLaunch` member functions. All other attributes will be treated as keyword arguments to that function. Call tables are executed in the order they appear in the launch file, and the result of calling their associated function will be stored under the call table's name.

    For example:

    .. code-block:: toml
        max_respawns = 3

        [node-config]
        func = "load_params"
        package = "my-package"
        configfile = "the-config.yml"

        [mynode]
        func = "node"
        package = "my-package"
        executable = "my-node"
        params = "$(node-config)"
        max_respawns = "$(max_respawns)"

    Executing this launchfile will first call :py:meth:`BetterLaunch.load_params` to locate a configuration file and store the result in the `node-config` key. Next, a node will be created by calling :py:meth:`BetterLaunch.node` with the call table's attributes. The `params` argument will be substituted to use the contents of the previously loaded config file. For convenience, since no name was specified the node will use the name of the table ("mynode").

    Any entry that is not a call table will be treated as a launch argument. Just like the results of call tables, these arguments can be used in substitutions (you may remember similar patterns from ROS1). There are a few special variants:
    - `$(<K>)` as above, this will resolve to a launch arg or call table result named <K>
    - `$(param <N> <P>)` will retrieve a parameter <P> from the *full* nodename <N>
    - `$(env <E> <D>)` will get the environment variable <E> (default to <D> if specified)
    - `$(eval <X>)` will treat <X> as a python expression to evaluate

    Substitutions can also be nested, in which case the innermost ones will be resolved first.

    For those functions in :py:class:`BetterLaunch` which are used as context objects (e.g. :py:meth:`BetterLaunch.group`, :py:meth:`BetterLaunch.compose`) you may provide a `children` attribute, which must be a dict of dicts. It's possible to use TOML's sutables for this like so:

    .. code-block:: toml

        [my_composer]
        func = "compose"

        [my_composer.children.talker]
        func = "component"
        package = "composition"
        plugin = "composition::Talker"

    In addition, any call table may contain an `if` and `unless` attribute to tie execution to a condition (which of course may contain substitutions).

    Lastly, there are a couple of special keys:
    - `bl_toml_format`: the better_launch TOML parser version your launch file was written for. Set this if the format has changed and you don't want to update your launch file. The current version is :py:data:`toml_format_version`.
    - `bl_eval_mode`: if and how `$(eval ...)` substitutions should be supported. `full`: regular eval. `literal`: only literals (uses :py:meth:`ast.literal_eval`). `none`: don't evaluate and return the substitution content verbatim.
    - `bl_ui`: default value for starting the UI
    - `bl_join`: whether to join the processes better_launch starts
    - `bl_colormode`: default colormode
    - `bl_screen_log_format`: default terminal output format
    - `bl_file_log_format`: default file log format
    - `bl_manage_foreign_nodes`: whether to show foreign nodes in the UI
    - `bl_keep_alive`: whether to keep running after the last node exits
    - `bl_allow_kwargs`: whether additional launch arguments are allowed

    Parameters
    ----------
    content : dict[str, Any]
        Contents of a TOML launch file.

    Returns
    -------
    dict[str, Any]
        The results of the executed calls.
    """
    toml: dict = load_toml(path)
    launch_args = _get_toml_launch_args(toml)
    docstring = toml.get("__comment__")

    toml_format = int(toml.get("bl_toml_format", toml_format_version))
    if toml_format != toml_format_version:
        print(f"Warning: TOML launch file has unexpected format version {toml_format}")

    eval_mode = toml.get("bl_eval_mode", "full")
    ui = toml.get("bl_ui", "false") in ("true", "enable", "1")
    join = toml.get("bl_join", True)
    colormode = Colormode[toml.get("bl_colormode", Colormode.DEFAULT.name)]
    screen_log_format = toml.get("bl_screen_log_format", None)
    file_log_format = toml.get("bl_file_log_format", None)
    manage_foreign_nodes = toml.get("bl_manage_foreign_nodes", False)
    keep_alive = toml.get("bl_keep_alive", False)
    allow_kwargs = toml.get("bl_allow_kwargs", True)

    def launch_func(*args, **kwargs):
        toml.update(kwargs)
        _execute_toml(toml, eval_mode=eval_mode)

    _exec_launch_func(
        launch_func,
        launch_args,
        docstring,
        ui=ui,
        join=join,
        colormode=colormode,
        screen_log_format=screen_log_format,
        file_log_format=file_log_format,
        manage_foreign_nodes=manage_foreign_nodes,
        keep_alive=keep_alive,
        allow_kwargs=allow_kwargs,
    )
