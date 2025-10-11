from typing import Any
import inspect
from pathlib import Path
import contextlib
import toml
import click

from better_launch import BetterLaunch
from .substitutions import apply_substitutions


toml_format_version = 1


# TODO remove substitutions from BetterLaunch?
def _execute_toml(content: dict[str, Any]) -> dict[str, Any]:
    """Execute a better_launch launchfile written in TOML.

    In better_launch TOML launch files, tables are also called `call tables`. A call table is a dict that has a  `func` key referring to one of the public :py:class:`BetterLaunch` member functions. All other attributes will be treated as keyword arguments to that function. Call tables are executed in the order they appear in the launch file, and the result of calling their associated function will be stored under the call table's name.

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

    Any non-table entry in the launch file will be treated as a launch argument. Just like the results of call tables these arguments can be used in substitutions (you may remember similar patterns from ROS1). There are a few special variants:
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
    if BetterLaunch.instance():
        raise RuntimeError("BetterLaunch has already been initialized")

    # Initialize the launcher instance
    bl = BetterLaunch.instance()
    valid_funcs = set(f for f in bl.__dict__.keys() if not f.startswith("_"))
    results = dict(bl.launch_args)

    # Not needed right now
    #bl_toml_format = int(content.get("bl_toml_format", toml_format_version))
    bl_eval_mode = content.get("bl_eval_mode", "full")

    def exec_request(key: str, req: dict) -> Any:
        for attr, val in req.items():
            req[attr] = apply_substitutions(val, results, eval_type=bl_eval_mode)

        if not req.get("if", True):
            return

        if req.get("unless", False):
            return

        # If not specified assume we're creating a node
        func_name = req["func"]
        if func_name not in valid_funcs:
            raise KeyError(f"func='{func_name}' is not a valid request")

        func = getattr(bl, func_name)

        # Where accepted the table key can be used as the name (e.g. of a node)
        if "name" in inspect.signature(func).parameters and "name" not in req:
            req["name"] = key

        # Call the function and store the result
        children = req.pop("children", None)
        res = func(**req)
        results[key] = res

        if children and isinstance(res, contextlib.AbstractContextManager):
            with res:
                # Must be a proper TOML subtable, we don't accept arrays of tables here
                if not isinstance(children, dict):
                    raise ValueError(f"Children of {key} must be specified as a dict")

                for subkey, child in children.items():
                    exec_request(key + "." + subkey, child)

    for key, val in content.items():
        if isinstance(val, dict):
            exec_request(key, val)
        else:
            results[key] = val

    return results


# TODO only works for comments that come after their key
class CustomTomlParser(toml.TomlDecoder):
    def __init__(self, _dict=dict):
        self.saved_comments = {}
        super(CustomTomlParser, self).__init__(_dict)

    def preserve_comment(self, line_no, key, comment, beginline):
        if line_no - 1 in self.saved_comments:
            # Bundle comments across multiple lines
            key, prev_comment, beginline = self.saved_comments[line_no - 1]
            comment = prev_comment + "\n" + comment
        
        self.saved_comments[line_no] = (key, comment, beginline)

    def embed_comments(self, idx, currentlevel):
        if idx - 1 not in self.saved_comments:
            return

        key, comment, beginline = self.saved_comments[idx - 1]
        current_val = currentlevel[key]
        if isinstance(current_val, toml.decoder.CommentValue):
            current_val.comment = comment
        else:
            currentlevel[key] = toml.decoder.CommentValue(currentlevel[key], comment, beginline,
                                            self._dict)


def launch_toml(path: str) -> None:
    decoder = toml.CustomTomlParser()
    content: dict = toml.load(path, decoder=decoder)
    launch_args = {}
    options = []
    docstrings = {}

    # In python's toml implementation, comments are associated with the previous key, but we want
    # them on the next key
    for key, comment, _ in decoder.saved_comments.values():
        docstrings[key] = comment

    # Collect the launch args
    for key, val in content.items():
        if isinstance(val, dict) and "func" in val:
            continue

        launch_args[key] = val

        options.append(
            click.Option(
                [f"--{key}"],
                type=type(val),
                default=val,
                show_default=True,
                help=docstrings.get(key),
            )
        )

    @click.pass_context
    def run(ctx: click.Context, *args, **kwargs):
        # TODO
        pass

    # TODO
    click_cmd = click.Command(Path(path).name, callback=run, params=options, help=launch_func_doc)

    bl = BetterLaunch(Path(path).name)
