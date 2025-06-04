from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungatec',
    maintainer_email='hungatec@oregonstate.edu',
    description='hw4: an exploration of parameters, launch files, and actions',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'oscope = hw4.oscope:basic_wave',
			'launcher = hw4.action_server:main',
			'listener = hw4.action_client:main',
			'canceled_listener = hw4.action_client:main_with_cancel',
        ],
    },
)
