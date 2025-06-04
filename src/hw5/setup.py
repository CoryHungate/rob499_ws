from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
		(os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
		

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungatec',
    maintainer_email='coryjhungate@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'filtered_laserscan = hw5.laser_scan_filter:main',
			'people_detector = hw5.people_detector:main',
			'personal_space = hw5.personal_space_visualizer:main',
			'intrusion_detector = hw5.intrusion_detector:main',
			
        ],
    },
)
