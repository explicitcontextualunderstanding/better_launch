from typing import Any, Literal, Callable
import os
from ast import literal_eval
from functools import partial
import yaml

from better_launch import BetterLaunch


_sentinel = object()


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


# $(param /myrobot/my_node rate)
def sub_param(full_node_name: str, param: str):
    # Try to delay ROS2 imports until we actually need them
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType

    bl = BetterLaunch.instance()

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
def sub_env(key: str, default: Any = _sentinel):
    if default != _sentinel:
        return os.environ.get(key, default)
    return os.environ[key]


# $(eval $(arg x) * 5)
def sub_eval(*args, context: dict, eval_type: Literal["full", "literal", "none"] = "full"):
    expr = " ".join(args)
    if eval_type == "full":
        return eval(expr, {}, dict(context) if context else {})
    elif eval_type == "literal":
        return literal_eval(expr)
    else:
        # eval was disabled
        return expr


def apply_substitutions(
    value: str,
    substitutions: dict[str, Callable] = None,
    # TODO turn into resolve function
    context: dict[str, Any] = None,
    *,
    eval_type: Literal["full", "literal", "none"] = "full",
) -> str:
    """Applies substitutions to a string.

    Substitution strings are expected to follow the "ROS1 pattern": `$(key *args)`, where `key` is a substitution type and `*args` are additional arguments to the substitution handler.

    If no other substitutions are specified, this function will handle the following ones:
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
    # TODO

    Raises
    ------
    ValueError
        If the input string contains unbalanced parentheses or quotes.
    """
    if not context:
        context = {}

    if not substitutions:
        _eval = partial(sub_eval, context=context, eval_type=eval_type)

        substitutions = {
            "param": sub_param,
            "env": sub_env,
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
