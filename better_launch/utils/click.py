from typing import Any, Type, Iterable, Callable
import os
from dataclasses import dataclass
import click

from better_launch.utils.better_logging import Colormode


@dataclass
class LaunchArg:
    name: str
    ptype: Type
    default: Any = None
    description: str = None


@dataclass
class Overrides:
    ui: bool = False
    colormode = Colormode.DEFAULT
    screen_log_format: str = None
    file_log_format: str = None

    def __init__(
        self,
        ui: bool = False,
        colormode: Colormode = Colormode.DEFAULT,
        screen_log_format: str = None,
        file_log_format: str = None,
    ):
        env_ui = os.environ.get("BL_UI_OVERRIDE", str(ui)).lower()
        self.ui = env_ui in ("enable", "true", "1")

        env_colormode = os.environ.get("BL_COLORMODE_OVERRIDE", colormode.name)
        self.colormode = Colormode[env_colormode.upper()]

        self.screen_log_format = os.environ.get(
            "BL_SCREEN_LOG_FORMAT_OVERRIDE", screen_log_format
        )
        self.file_log_format = os.environ.get(
            "BL_FILE_LOG_FORMAT_OVERRIDE", file_log_format
        )


def get_click_options(launch_args: Iterable[LaunchArg]) -> list[click.Option]:
    options = []
    for arg in launch_args:
        options.append(
            click.Option(
                [f"--{arg.name}"],
                type=arg.ptype,
                default=arg.default,
                show_default=True,
                help=arg.description,
            )
        )

    return options


def get_click_overrides(overrides: Overrides) -> list[click.Option]:
    # Additional overrides for launch arguments
    def set_override_ui(ctx: click.Context, param: click.Parameter, value: str):
        if value != "unset":
            overrides.ui = value == "enable"
        return value

    def set_override_colormode(ctx: click.Context, param: click.Parameter, value: str):
        if value:
            overrides.colormode = Colormode[value.upper()]
        return value

    # NOTE these should be mirrored in the bl script
    options = [
        click.Option(
            ["--bl_ui_override"],
            type=click.types.Choice(
                ["enable", "disable", "unset"], case_sensitive=False
            ),
            show_choices=True,
            default="unset",
            help="Override to enable/disable the terminal UI",
            expose_value=False,  # not passed to our run method
            callback=set_override_ui,
        ),
        click.Option(
            ["--bl_colormode_override"],
            type=click.types.Choice([c.name for c in Colormode], case_sensitive=False),
            show_choices=True,
            default=Colormode.DEFAULT.name,
            help="Override the logging color mode",
            expose_value=False,
            callback=set_override_colormode,
        ),
    ]

    return options


def get_click_launch_command(
    cmd_name: str,
    launch_func: Callable,
    options: Iterable[click.Option],
    cmd_help: str = None,
    *,
    allow_kwargs: bool = False,
) -> click.Command:
    click_cmd = click.Command(
        cmd_name, callback=launch_func, params=options, help=cmd_help
    )

    if allow_kwargs:
        click_cmd.allow_extra_args = True
        click_cmd.ignore_unknown_options = True

    return click_cmd
