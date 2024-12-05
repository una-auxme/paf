#!/usr/bin/env python
import os
from setuptools import setup
from setuptools.extension import Extension
from Cython.Build import cythonize
from catkin_pkg.python_setup import generate_distutils_setup

import numpy as np

setup_args = generate_distutils_setup(
    packages=["mapping_common"], package_dir={"": "src"}
)

mapping_common_files = []
for root, dirs, files in os.walk("src/mapping_common"):
    for file in files:
        if file.endswith(".py"):
            mapping_common_files.append(os.path.join(root, file))

extensions = [
    Extension(
        name="mapping_common_c",
        sources=mapping_common_files,
        # https://cython.readthedocs.io/en/latest/src/userguide/numpy_tutorial.html
        include_dirs=[np.get_include()],
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
    ),
]

setup(ext_modules=cythonize(extensions), **setup_args)
