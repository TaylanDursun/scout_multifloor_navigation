import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'team4_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        # package manifest
        (f'share/{package_name}', ['package.xml']),
        # launch + config install
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diakamos',
    maintainer_email='diakamos@todo.todo',
    description='Team 4 bringup package (launch/config utilities)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ileride node script eklersen buraya yazarsın
            # 'floor_decider = team4_bringup.floor_decider:main',
        ],
    },
)

