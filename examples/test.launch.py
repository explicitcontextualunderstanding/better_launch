#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this, convenience
#from better_launch.declarative import launch_toml


@launch_this
def test():
    bl = BetterLaunch()
    convenience.spawn_controller_manager(name="my_controller")


#if __name__ == "__main__":
#    path = "src/better_launch/examples/12_declarative.launch.toml"
#    launch_toml(path)
