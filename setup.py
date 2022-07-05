import os
from glob import glob
from setuptools import setup

package_name = "kone_ros_api"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="jun_hao_chng@cgh.com.sg",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # "koneNode = kone_ros_api.koneNode:main",
            "koneAdaptor_v2 = kone_ros_api.koneAdaptor_v2:main",
            "koneNode_v2 = kone_ros_api.koneNode_v2:main",
            "koneAdaptor = kone_ros_api.koneAdaptor:main",
        ],
    },
)
