#!/usr/bin/env python
import os
from setuptools import setup
from setuptools.extension import Extension
from Cython.Build import cythonize

import numpy as np

extensions = [
    Extension(
        "*",
        sources=["mapping_common/*.py"],
        # https://cython.readthedocs.io/en/latest/src/userguide/numpy_tutorial.html
        include_dirs=[np.get_include()],
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
    ),
]


def is_debug_enabled() -> bool:
    debug_enabled_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), ".debug_enabled"
    )
    if os.path.exists(debug_enabled_file):
        with open(debug_enabled_file) as f:
            line = f.readline().strip().lower()
            return line == "true"
    return False


def main():
    if is_debug_enabled():
        package_setup = {"packages": ["mapping_common"]}
        print("Installing mapping_common in debug (pure python) mode")
    else:
        package_setup = {
            "ext_modules": cythonize(extensions, language_level="3", annotate=True)
        }
        print("Installing mapping_common in compiled cython mode")

    setup(name="mapping_common", version="0.0.1", **package_setup)


if __name__ == "__main__":
    main()
