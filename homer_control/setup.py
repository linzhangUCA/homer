import os
from glob import glob
from setuptools import setup

package_name = "homer_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="linzhanguca",
    maintainer_email="lzhang12@uca.edu",
    description="Control HomeR's velocity and publish odometry",
    license="GPL3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = homer_control.driver:main"],
    },
)
