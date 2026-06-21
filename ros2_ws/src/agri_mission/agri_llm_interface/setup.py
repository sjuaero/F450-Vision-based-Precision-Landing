from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agri_llm_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'prompts'),
            glob('agri_llm_interface/prompts/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='LLM interface for agricultural mission commands',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = agri_llm_interface.llm_node:main',
        ],
    },
)
