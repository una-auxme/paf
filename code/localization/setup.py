import os
from glob import glob
from setuptools import find_packages, setup

package_name = "localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="peter",
    maintainer_email="peter.viechter@student.uni-augsburg.de",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_state_publisher = localization.ekf_state_publisher:main",
            "gps_debug_node = localization.gps_debug_node:main",
            "gps_transform = localization.gps_transform:main",
            "kalman_filter = localization.kalman_filter:main",
            "odometry_fusion = localization.odometry_fusion:main",
            "position_heading_publisher_node = "
            "localization.position_heading_publisher_node:main",
            "sensor_covariance_fusion = localization.sensor_covariance_fusion:main",
        ],
    },
)
