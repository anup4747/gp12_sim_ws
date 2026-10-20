import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gp_12_trail'

# Include all mesh, URDF/Xacro, and RViz files so they are available in the install space.
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
]

for root in ['urdf', 'meshes']:
    for path in glob(os.path.join(root, '**', '*'), recursive=True):
        if os.path.isfile(path):
            rel_dir = os.path.relpath(os.path.dirname(path), '.')
            dest = os.path.join('share', package_name, rel_dir)
            data_files.append((dest, [path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    scripts=['script/pick_place.py'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janhavi',
    maintainer_email='janhavi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
