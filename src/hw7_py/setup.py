from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw7_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungatec',
    maintainer_email='hungatec@oregonstate.edu',
    description='python node, launch file, and RVIZ2 config file for hw7',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'object_detector = hw7_py.object_detector:main',
        ],
    },
)
