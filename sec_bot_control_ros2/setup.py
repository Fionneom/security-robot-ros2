import os
from glob import glob
from setuptools import setup

package_name = 'sec_bot_control_ros2'
submodules = "sec_bot_control_ros2/submodules"

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name, submodules],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',

    entry_points={
        'console_scripts': [
            'odom_publisher = sec_bot_control_ros2.odom_publisher:main',
            'robot_controller = sec_bot_control_ros2.robot_controller:main',
            'hardware_interface = sec_bot_control_ros2.hardware_interface:main',
        ],
    },
)