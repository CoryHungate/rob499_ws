from setuptools import find_packages, setup

package_name = 'hw3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungatec',
    maintainer_email='hungatec@oregonstate.edu',
    description='homework 3: simple coding exercise as an introduction to services in ROS2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'data_sender = hw3.data_sender:main',
			'data_receiver = hw3.data_receiver:main',
        ],
    },
)
