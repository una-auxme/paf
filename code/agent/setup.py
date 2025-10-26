import os
from glob import glob
from setuptools import find_packages, setup

package_name = "agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="peter",
    maintainer_email="peter.viechter@student.uni-augsburg.de",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    scripts=["scripts/launch_agent.sh"],
    entry_points={
        "console_scripts": [
            "data_management_node = agent.data_management_node:main",
        ],
    },
)
