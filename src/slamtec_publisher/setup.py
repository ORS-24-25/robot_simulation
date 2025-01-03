from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slamtec_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('src/' + package_name + '/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sstevenson',
    maintainer_email='samirdstevenson@gmail.com',
    description='Package that deals with all interactions with SLAMTEC M2M2 Mapper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slamtec_publisher = slamtec_publisher.slamtec_publisher:main'
        ],
    },
)
