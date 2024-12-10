#!/usr/bin/env python
import os
from setuptools import setup
from setuptools.extension import Extension
from Cython.Build import cythonize

import numpy as np

mapping_common_files = []
for root, dirs, files in os.walk("mapping_common"):
    for file in files:
        if file.endswith(".py"):
            mapping_common_files.append(os.path.join(root, file))

extensions = [
    Extension(
        name="mapping_common",
        sources=mapping_common_files,
        # https://cython.readthedocs.io/en/latest/src/userguide/numpy_tutorial.html
        include_dirs=[np.get_include()],
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
    ),
]


def main():
    setup(
        name="mapping_common",
        version="0.0.1",
        ext_modules=cythonize(extensions, language_level="3"),
    )


if __name__ == "__main__":
    main()
