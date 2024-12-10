#!/usr/bin/env python
from argparse import ArgumentParser
import os
import sys
import setuptools
from setuptools import setup
import setuptools.command
import setuptools.command.build
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


# # Override build command for setting the build directory
# class BuildCommand(setuptools.command.build.build):
#     user_options = [("build_dir_override=", None, "build directory")]

#     def initialize_options(self):
#         super().initialize_options()
#         self.build_dir_override = None

#     def finalize_options(self):
#         print("Blas")
#         if self.build_dir_override is not None:
#             self.build_base = self.build_dir_override
#             print(f"Using custom build dir: {self.build_base}")
#         super().finalize_options()


def main():
    # parser = ArgumentParser(
    #     prog="mapping_common setup",
    # )
    # parser.add_argument("--build_dir ", required=False, type=str)
    # # parser.add_argument("--debug_port", required=False, type=int)
    # # parser.add_argument("--debug_host", default=default_host, type=str)
    # # parser.add_argument("--debug_wait", action="store_true")
    # args, unknown_args = parser.parse_known_args(sys.argv)
    # if "build_dir" in args:
    #     global BUILD_DIR
    #     BUILD_DIR = args.build_dir

    setup(
        name="mapping_common",
        packages=["mapping_common"],
        package_dir={"mapping_common": "mapping_common"},
        package_data={"src/mapping_common": ["*.pxd", "*.py"]},
        ext_modules=cythonize(extensions, language_level="3"),
        # cmdclass={"build": BuildCommand},
    )


if __name__ == "__main__":
    main()
