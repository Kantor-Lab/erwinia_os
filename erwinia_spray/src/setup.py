from setuptools import find_packages, setup

package_name = 'spray_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='appleseed_labs',
    maintainer_email='appleseed_labs@todo.todo',
    description='ROS2 service wrapper for an Arduino-controlled spray unit via serial.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spray_node = spray_control.spray_node:main',
        ],
    },
)