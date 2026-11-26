from glob import glob

from setuptools import find_packages, setup

package_name = 'excercise_1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grininger, Kiss',
    maintainer_email='todo@todo.todo',
    description='ARS3IL Exercise 1 - odometry and laser scanner statistics (TurtleBot3)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_odometry = excercise_1.my_odometry_node:main',
            'trajectory_logger = excercise_1.trajectory_logger_node:main',
            'scan_statistics = excercise_1.scan_statistics_node:main',
            'scan_sum = excercise_1.scan_sum_node:main',
            'tf_filter = excercise_1.tf_filter_node:main',
        ],
    },
)
