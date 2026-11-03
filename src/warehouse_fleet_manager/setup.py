import os
from glob import glob

from setuptools import setup

package_name = 'warehouse_fleet_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ranjith',
    maintainer_email='you@example.com',
    description='Fleet manager: task queue, closest-available-healthy-robot allocation with battery check, fleet status.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_manager = warehouse_fleet_manager.manager:main',
            'submit_random_tasks = warehouse_fleet_manager.submit_random_tasks:main',
            'fleet_monitor = warehouse_fleet_manager.monitor:main',
        ],
    },
)
