from setuptools import find_packages, setup
from glob import glob
import os

package_name = "schunk_fts_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "test.*", "tests", "*.tests", "*.tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), [package_name + "/driver.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "black==26.5.1",
        "certifi==2026.6.17",
        "charset-normalizer==3.4.7",
        "click==8.4.2",
        "exceptiongroup==1.3.1",
        "idna==3.18",
        "iniconfig==2.3.0",
        "lark==1.3.1",
        "mypy_extensions==1.1.0",
        "numpy==2.2.6",
        "packaging==26.2",
        "pathspec==1.1.1",
        "platformdirs==4.10.0",
        "pluggy==1.6.0",
        "pytest==9.0.3",
        "pytest-repeat==0.9.4",
        "PyYAML==6.0.3",
        "requests==2.34.2",
        "tomli==2.4.1",
        "typing_extensions==4.15.0",
        "urllib3==2.7.0",
    ],
    zip_safe=True,
    maintainer="Fabian Reinwald",
    maintainer_email="fabian.reinwald@de.schunk.com",
    description="ROS2 driver for SCHUNK's force-torque sensors",
    license="GPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
