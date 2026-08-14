from setuptools import find_packages, setup

package_name = "roboreg_base"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="Data collection utilities for the ROS 2 roboreg integration..",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "broadcaster = scripts.broadcaster_node:main",
        ],
    },
)
