from typing import Any
import inspect
from pathlib import Path
import contextlib
from ast import literal_eval
import logging
import click

from better_launch import BetterLaunch
from better_launch.utils.better_logging import (
    Colormode,
    init_logging,
)
from better_launch.ros import logging as roslog

from .toml_parser import load as load_toml
from .substitutions import apply_substitutions


toml_format_version = 1


def _execute_toml(toml: dict[str, Any]) -> dict[str, Any]:
    """Execute each call table and apply substitutions."""
    if BetterLaunch.instance():
        raise RuntimeError("BetterLaunch has already been initialized")

    # Initialize the launcher instance
    bl = BetterLaunch.instance()
    valid_funcs = set(f for f in bl.__dict__.keys() if not f.startswith("_"))
    results = dict(bl.launch_args)

    # Not needed right now
    # bl_toml_format = int(content.get("bl_toml_format", toml_format_version))
    bl_eval_mode = toml.get("bl_eval_mode", "full")

    def exec_request(key: str, req: dict) -> Any:
        if "func" not in req:
            return
        
        for attr, val in req.items():
            req[attr] = apply_substitutions(val, results, eval_type=bl_eval_mode)

        if not req.pop("if", True):
            return

        if req.pop("unless", False):
            return

        # If not specified assume we're creating a node
        func_name = req["func"]
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
                logging.getLogger().warning(f"{req} has 'children' key, but did not result in a context object - children will be ignored")
                return

            with res:
                # Must be a proper TOML subtable, we don't accept arrays of tables here
                if not isinstance(children, dict):
                    raise ValueError(f"Children of {key} must be specified as a dict")

                for subkey, child in children.items():
                    exec_request(key + "." + subkey, child)

    for key, val in toml.items():
        if isinstance(val, dict):
            exec_request(key, val)
        else:
            results[key] = val

    return results


def launch_toml(
    path: str, 
    *,
    ui: bool = False,
    join: bool = True,
    screen_log_format: str = None,
    file_log_format: str = None,
    colormode: Colormode = Colormode.DEFAULT,
    manage_foreign_nodes: bool = False,
    keep_alive: bool = False,
) -> None:
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
    - `bl_toml_format`: the better_launch TOML parser version your launch file was written for. Set this if the format has changed and you don't want to update your launch file. The current version is `1`.
    - `bl_eval_mode`: if and how `$(eval ...)` substitutions should be supported. `full`: regular eval. `literal`: only literals (uses :py:meth:`ast.literal_eval`). `none`: don't evaluate and return the substitution content verbatim.

    Parameters
    ----------
    content : dict[str, Any]
        Contents of a TOML launch file.

    Returns
    -------
    dict[str, Any]
        The results of the executed calls.
    """
    # FIXME a lot more to do, see launch_this for details (sigint setup, read env variables, etc)

    toml: dict = load_toml(path)
    options = []

    # Collect the launch args
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

        docstring = toml.get(f"__comment_{key}__")

        options.append(
            click.Option(
                [f"--{key}"],
                type=ptype,
                default=default,
                show_default=True,
                help=docstring,
            )
        )

    # TODO move click setup to utility function
    # Additional overrides for launch arguments
    def click_ui_override(ctx: click.Context, param: click.Parameter, value: str):
        if value != "unset":
            nonlocal ui
            ui = value == "enable"
        return value

    def click_colormode_override(
        ctx: click.Context, param: click.Parameter, value: str
    ):
        if value:
            nonlocal colormode
            colormode = Colormode[value.upper()]
        return value

    # NOTE these should be mirrored in the bl script
    options.extend(
        [
            click.Option(
                ["--bl_ui_override"],
                type=click.types.Choice(
                    ["enable", "disable", "unset"], case_sensitive=False
                ),
                show_choices=True,
                default="unset",
                help="Override to enable/disable the terminal UI",
                expose_value=False,  # not passed to our run method
                callback=click_ui_override,
            ),
            click.Option(
                ["--bl_colormode_override"],
                type=click.types.Choice([c.name for c in Colormode], case_sensitive=False),
                show_choices=True,
                default=Colormode.DEFAULT.name,
                help="Override the logging color mode",
                expose_value=False,
                callback=click_colormode_override,
            ),
        ]
    )

    def launch_wrapper():
        # Execute the launch function!
        try:
            _execute_toml(toml)
        except Exception as e:
            bl = BetterLaunch.instance()
            if bl and not bl.is_shutdown:
                bl.shutdown(f"Exception in launch file: {e}")

            raise

        # Retrieve the BetterLaunch singleton
        bl = BetterLaunch()

        # The UI will manage spinning itself
        if join and not ui:
            bl.spin(exit_with_last_node=not keep_alive)

    @click.pass_context
    def run(ctx: click.Context, *args, **kwargs):
        if args:
            raise ValueError(
                "Positional arguments are not supported for toml launch files"
            )

        init_logging(
            roslog.launch_config, screen_log_format, file_log_format, colormode
        )

        # Apply the launch arguments from click
        toml.update(kwargs)
        
        assert (
            len(ctx.args) % 2 == 0
        ), "All arguments need to be '--<key> <value>' tuples"

        # Add additional launch arguments
        for (i,) in range(0, len(ctx.args), 2):
            (key,) = ctx.args[i]
            if not key.startswith("-"):
                raise ValueError("Argument keys must start with a dash")

            val = ctx.args[i + 1]
            try:
                val = literal_eval(val)
            except Exception:
                # Keep val as a string
                pass

            toml[key.strip("-")] = val

        BetterLaunch._launch_func_args = toml
        
        if ui:
            from better_launch.tui.better_tui import BetterTui

            app = BetterTui(
                launch_wrapper, 
                manage_foreign_nodes=manage_foreign_nodes,
                keep_alive=keep_alive,
            )
            app.run()
        else:
            launch_wrapper()

    launch_doc = toml.get("__comment__")
    click_cmd = click.Command(
        Path(path).name, callback=run, params=options, help=launch_doc
    )
    
    # TODO should we allow arbitrary launch args not specified in the toml?
    #click_cmd.allow_extra_args = True
    #click_cmd.ignore_unknown_options = True

    try:
        click_cmd.main()
    except SystemExit as e:
        if e.code != 0:
            raise
