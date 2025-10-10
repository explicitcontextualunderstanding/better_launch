from typing import Any, Literal
import os
import inspect
from ast import literal_eval
from pathlib import Path
import contextlib
import yaml
import toml

from better_launch import BetterLaunch


_sentinel = object()
toml_format_version = 1


class SubstitutionError(ValueError):
    """Exception type that will be thrown by substitution handlers."""

    pass


def _parse_substitutions(s: str) -> list[list | str]:
    """Parses a string containing substitution tokens into a list of lists and strings.

    Parameters
    ----------
    s : str
        The input string to parse.

    Returns
    -------
    list[list | str]
        A list containing unchanged strings and nested lists of strings/lists. These nested lists
        will consist of the substitution key and its arguments, which again may be lists.

    Raises
    ------
    ValueError
        If the input string contains unbalanced parentheses or quotes.
    """

    def tokenize(s):
        i = 0
        n = len(s)
        while i < n:
            if s[i].isspace():
                i += 1
                continue
            elif s[i] == "$" and i + 1 < n and s[i + 1] == "(":
                yield "$("
                i += 2
            elif s[i] == ")":
                yield ")"
                i += 1
            elif s[i] in "\"'":
                quote = s[i]
                i += 1
                start = i
                while i < n:
                    if s[i] == quote and s[i - 1] != "\\":
                        break
                    i += 1
                else:
                    raise ValueError("Missing closing quote")
                yield s[start:i]
                i += 1  # skip closing quote
            else:
                start = i
                while i < n and not s[i].isspace() and s[i] not in "$()":
                    i += 1
                yield s[start:i]

    def parse(tokens):
        stack = []
        current = []
        is_key = False
        for tok in tokens:
            if tok == "$(":
                stack.append(current)
                current = []
                # Next token should be marked as a substitution key
                is_key = True
            elif tok == ")":
                if not stack:
                    raise ValueError("Unbalanced )")
                completed = current
                current = stack.pop()
                current.append(completed)
            else:
                if is_key:
                    tok = "$" + tok
                    is_key = False
                current.append(tok)
        
        if stack:
            raise ValueError("Unbalanced $(")

        return current

    return parse(tokenize(s))


def _apply_substitutions(
    value: str,
    # TODO turn into resolve function
    context: dict[str, Any] = None,
    *,
    eval_type: Literal["full", "literal", "none"] = "full",
) -> str:
    """Applies substitutions to a string.

    Substitution strings are expected to follow the "ROS1 pattern": `$(key *args)`, where `key` is a substitution type and `*args` are additional arguments to the substitution handler.

    This function will handle the following substitutions:
    - $(param ...)
    - $(env ...)
    - $(eval ...)

    Substitutions other than those above will be looked up in the provided `context` dict.

    Parameters
    ----------
    value : str
        A string that may contain substitution tokens.
    context : dict[str, Any]
        A dict containing additional values the substitution may use.
    eval_type : Literal["full", "literal", "none"], optional
        If and to what extend `eval` should be allowed.

    Returns
    -------
    str
        The input string with all substitution tokens handled.

    Raises
    ------
    ValueError
        If the input string contains unbalanced parentheses or quotes.
    """
    # $(param /myrobot/my_node rate)
    def _param(full_node_name: str, param: str):
        # Try to delay ROS2 imports until we actually need them
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType

        srv = bl.shared_node.create_client(
            GetParameters, f"{full_node_name}/get_parameters"
        )

        if not srv.wait_for_service(5.0):
            raise SubstitutionError("Failed to wait for node parameter service")

        req = GetParameters.Request()
        req.names = [param]
        res = srv.call(req)

        if len(res.values) != 1:
            raise SubstitutionError(
                f"Failed to retrieve parameter {param} from {full_node_name}"
            )

        try:
            value = yaml.safe_load(res.values[0])
        except Exception:
            # Treat it as a string
            value = str(res.values[0])

        # Importing get_value from ros2param will increase memory footprint by ~5MB, so diy
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        elif value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        elif value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        elif value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(value.byte_array_value)
        elif value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        elif value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        elif value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        elif value.type == ParameterType.PARAMETER_NOT_SET:
            return None

        return None

    # $(env ROS_DISTRO)
    def _env(key: str, default: Any = _sentinel):
        if default != _sentinel:
            return os.environ.get(key, default)
        return os.environ[key]

    # $(eval $(arg x) * 5)
    def _eval(*args):
        expr = " ".join(args)
        if eval_type == "full":
            return eval(expr, {}, dict(context))
        elif eval_type == "literal":
            return literal_eval(expr)
        else:
            # eval was disabled
            return expr

    bl = BetterLaunch.instance()
    if not context:
        context = {}

    substitutions = {
        "param": _param,
        "env": _env,
        "eval": _eval,
    }

    # This should only raise if the value contains "$(" AND has invalid syntax
    parsed = _parse_substitutions(value)

    # Evaluate the substitutions
    def delve(node: list | str) -> str:
        if isinstance(node, list):
            # Evaluate nested elements first
            evaluated = [delve(token) for token in node]
            sub_key, *sub_args = evaluated

            # Substitution keys will start with a $ (see parse() above)
            if sub_key.startswith("$"):
                sub_key = sub_key[1:]

                if sub_key in substitutions:
                    return substitutions[sub_key](*sub_args)

                # If the key is not a substitution key, assume it's the name of a launch
                # arg or call result
                return context[sub_key]
            else:
                return " ".join(evaluated)
        else:
            return node

    return delve(parsed)


# TODO remove substitutions from BetterLaunch?
def launch_toml(path: str) -> dict[str, Any]:
    """Execute a better_launch launchfile written in TOML.

    In better_launch TOML launch files, tables are also called `call tables`. Each call table may specify a `func` key (which defaults to "node" if omitted) referring to one of the public :py:class:`BetterLaunch` member functions. All other attributes will be treated as keyword arguments to that function. Call tables are executed in the order they appear in the launch file, and the result of calling their associated function will be stored under the call table's name.

    For example:

    .. code-block:: toml
        max_respawns = 3

        [node-config]
        func = "load_params"
        package = "my-package"
        configfile = "the-config.yml"

        [mynode]
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
    path : str
        Path to the toml file.

    Returns
    -------
    dict[str, Any]
        The results of the executed calls.
    """
    if BetterLaunch.instance():
        raise RuntimeError("BetterLaunch has already been initialized")

    # Initialize the launcher instance
    content: dict = toml.load(path)
    bl = BetterLaunch(Path(path).name)
    valid_funcs = set(f for f in bl.__dict__.keys() if not f.startswith("_"))
    results = dict(bl.launch_args)

    # Not needed right now
    #bl_toml_format = int(content.get("bl_toml_format", toml_format_version))
    bl_eval_mode = content.get("bl_eval_mode", "full")

    def exec_request(key: str, req: dict) -> Any:
        for attr, val in req.items():
            req[attr] = _apply_substitutions(val, results, eval_type=bl_eval_mode)

        if not req.get("if", True):
            return

        if req.get("unless", False):
            return

        # If not specified assume we're creating a node
        func_name = req.pop("func", "node")
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
