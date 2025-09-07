import os
from glob import glob
from setuptools import setup

package_name = "perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=["perception", "traffic_light_detection"],
    package_dir={
        "traffic_light_detection": os.path.join("traffic_light_detection", "src"),
    },
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
            "lane_position = perception.lane_position:main",
            "Lanedetection_node = perception.Lanedetection_node:main",
            "lidar_distance = perception.lidar_distance:main",
            "radar_node = perception.radar_node:main",
            "traffic_light_node = perception.traffic_light_node:main",
            "vision_node = perception.vision_node:main",
        ],
    },
)
