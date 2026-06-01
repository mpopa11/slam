from setuptools import find_packages, setup
import glob
import os

package_name = 'basic_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mihai Popa',
    maintainer_email='mihai.119.popa@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = basic_slam.imu_node:main',
            'imu_odom = basic_slam.imu_odom:main',
            'map_node = basic_slam.map_node:main',
            'odom_cov = basic_slam.odom_cov:main',
            'metrics = basic_slam.metrics:main',
            'ground_truth = basic_slam.ground_truth:main'
        ],
    },
)
