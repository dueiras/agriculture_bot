from setuptools import setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/params", glob("params/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/rviz2", glob("rviz2/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo Eiras',
    maintainer_email='dueiras@gmail.com',
    description='Global Navigation with NAV2 for Carter Robot in Isaac Sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'SetNavigationGoal = navigation.navigate_to_pose:main',
            'navigate_through_poses = navigation.navigate_through_poses:main'
        ],
    },
)
