from setuptools import setup
from glob import glob
import os

package_name = 'warehouse_gazebo'


def collect_files(directory):
    data_files = []
    for root, dirs, files in os.walk(directory):
        # Never install bytecode caches (created when tests import scripts).
        dirs[:] = [d for d in dirs if d != '__pycache__']
        files = [f for f in files if not f.endswith('.pyc')]
        if files:
            install_dir = os.path.join('share', package_name, root)
            data_files.append((install_dir, [os.path.join(root, f) for f in files]))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
]

data_files += collect_files('models')
data_files += collect_files('markers')
data_files += collect_files('scripts')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owl',
    maintainer_email='zesmaeili85@gmail.com',
    description='Warehouse Gazebo simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
