from setuptools import find_packages, setup

package_name = 'hw2'

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
    maintainer='Cory Hungate',
    maintainer_email='hungatec@oregonstate.edu',
    description='TODO: a fairly simple subscriber-publisher that generates a sine wave' \
            'this was created as a "get to know ROS2 example for rob499',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'oscope = hw2.oscope:basic_wave',
            'slow_wave = hw2.oscope:long_wave',
            'fast_wave = hw2.oscope:short_wave',
            'limiter = py_pubsub.limiter:main',
        ],
    },
)
