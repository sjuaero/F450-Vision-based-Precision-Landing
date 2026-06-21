import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agri_aruco_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ROS2가 찾는 위치: lib/<package_name>/
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ArUco marker detector for precision landing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = agri_aruco_detector.aruco_detector_node:main',
            'landing_pad_detector = agri_aruco_detector.landing_pad_detector_node:main',
        ],
    },
)
