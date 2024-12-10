#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


# setup_args = generate_distutils_setup(
#     packages=["mapping_common"],
#     package_dir={"": "src", "mapping_common": "src/mapping_common"},
#     package_data={"src/mapping_common": ["*.py"]},
# )

setup_args = generate_distutils_setup(
    package_dir={"": "src"},
)


setup(**setup_args)
