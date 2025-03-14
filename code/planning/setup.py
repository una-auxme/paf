#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["behavior_agent", "global_planner", "local_planner"],
    package_dir={"": "src"},
)
setup(**setup_args)
