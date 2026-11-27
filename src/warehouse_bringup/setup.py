from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warehouse_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owl',
    maintainer_email='zesmaeili85@gmail.com',
    description='Bringup package for the autonomous warehouse robot system.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'warehouse_autonomy = warehouse_bringup.warehouse_autonomy_node:main',
        ],
    },
)
