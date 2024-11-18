from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ors_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/Robot_Base.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/robot_base.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebajit3',
    maintainer_email='ebajit3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
