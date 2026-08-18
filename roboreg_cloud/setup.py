import glob

from setuptools import find_packages, setup

package_name = "roboreg_cloud"

setup(
    name=package_name,
    version="0.3.0",
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
    maintainer_email="m.huber_1994@hotmail.de",
    description="Implements roboreg_base and acts as a client to a hosted roboreg server.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "roboreg_monocular = roboreg_cloud.cli.roboreg_monocular:main",
            "roboreg_stereo = roboreg_cloud.cli.roboreg_stereo:main",
        ],
    },
)
