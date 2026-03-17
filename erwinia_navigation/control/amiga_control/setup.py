from setuptools import find_packages, setup

package_name = "amiga_control"

setup(
    name=package_name,
    version="0.0.0",  # See package.xml
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Francisco Yandun",
    maintainer_email="fyandun@andrew.cmu.edu",
    description="Nodes interface with an Amiga's CAN network",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"amiga_control = {package_name}.amiga_control:main",
        ],
    },
)
