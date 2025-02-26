import glob

from setuptools import find_packages, setup

package_name = "roboreg_nodes"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*")),
        ("share/" + package_name + "/launch", glob.glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="ROS 2 integration of the mesh registration library",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "monocular_depth = scripts.monocular_depth_reg:main",
            "stereo_depth = scripts.stereo_depth_reg:main",
            "broadcaster = scripts.broadcaster_node:main",
        ],
    },
)
