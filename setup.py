from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prajj',
    maintainer_email='prajj@todo.todo',
    description='AI Warehouse Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_mapper = warehouse_robot.image_mapper:main',
            'task_planner = warehouse_robot.task_planner:main',
            'navigator = warehouse_robot.navigator:main',
            'dashboard = warehouse_robot.dashboard_server:main',
            'pybullet_bridge = warehouse_robot.pybullet_bridge:main',
        ],
    },
)