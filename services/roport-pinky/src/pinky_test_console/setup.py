from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pinky_test_console'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/web', glob(os.path.join('web', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mindy',
    maintainer_email='maintainer@example.com',
    description='Pinky Pro test web console (SLAM, Nav2, teleop, memory status)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'web_server = pinky_test_console.web_server:main',
        ],
    },
)
