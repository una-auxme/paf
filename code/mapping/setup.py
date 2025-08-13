import os
from glob import glob
from setuptools import find_packages, setup
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


package_name = "mapping"


def main():
    packages = find_packages(exclude=["test"])
    ext_modules = {}
    if is_debug_enabled():
        # print("Installing mapping_common in debug (pure python) mode",
        #       file=sys.stderr)
        pass
    else:
        ext_modules = {
            "ext_modules": cythonize(
                extensions, language_level="3", annotate=True, quiet=True
            )
        }
        # Print statements are commented out and quiet=True, because colcon parses
        # the pip output and crashes if it finds stdout that it does not expect.
        # https://github.com/colcon/colcon-python-setup-py/issues/53
        # print("Installing mapping_common in compiled cython mode", file=sys.stderr)

    setup(
        name=package_name,
        version="0.0.0",
        packages=packages,
        data_files=[
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
            (os.path.join("share", package_name, "launch"), glob("launch/*")),
            (os.path.join("share", package_name, "config"), glob("config/*")),
            (os.path.join("share", package_name), [".debug_enabled"]),
        ],
        install_requires=["setuptools"],
        zip_safe=True,
        maintainer="peter",
        maintainer_email="peter.viechter@student.uni-augsburg.de",
        description="TODO: Package description",
        license="MIT",
        tests_require=["pytest"],
        entry_points={
            "console_scripts": [],
        },
        **ext_modules
    )


if __name__ == "__main__":
    main()
