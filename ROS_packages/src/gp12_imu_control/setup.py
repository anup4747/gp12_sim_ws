import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gp12_imu_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anup',
    maintainer_email='anupdtarwade@gmail.com',
    description='Android IMU integration with GP12 MoveIt',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hyperimu_bridge = gp12_imu_control.hyperimu_bridge:main',
            'imu_to_moveit = gp12_imu_control.imu_to_moveit:main'
        ],
    },
)
