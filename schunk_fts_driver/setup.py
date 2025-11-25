from setuptools import find_packages, setup
from glob import glob
import os

package_name = "schunk_fts_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), [package_name + "/driver.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "black==25.1.0",
        "certifi==2025.4.26",
        "charset-normalizer==3.4.2",
        "click==8.2.1",
        "exceptiongroup==1.3.0",
        "idna==3.10",
        "iniconfig==2.1.0",
        "lark==1.2.2",
        "mypy_extensions==1.1.0",
        "numpy==2.2.6",
        "packaging==25.0",
        "pathspec==0.12.1",
        "platformdirs==4.3.8",
        "pluggy==1.5.0",
        "pytest==8.3.5",
        "pytest-repeat==0.9.4",
        "PyYAML==6.0.2",
        "requests==2.32.4",
        "tomli==2.2.1",
        "typing_extensions==4.13.2",
        "urllib3==2.5.0",
    ],
    zip_safe=True,
    maintainer="stefan",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="ROS2 driver for SCHUNK`s force-torque sensors",
    license="GPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
