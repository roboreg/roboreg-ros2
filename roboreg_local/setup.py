import glob

from setuptools import find_packages, setup

package_name = "roboreg_local"

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
    maintainer_email="m.huber_1994@hotmail.de",
    description="Implements roboreg_base using the roboreg Python library locally.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "roboreg_monocular = scripts.roboreg_monocular:main",
            "roboreg_stereo = scripts.roboreg_stereo:main",
        ],
    },
)
