from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roport_gz_world'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.xml'))),
        ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*.sdf'))),
        ('share/' + package_name + '/layout', glob(os.path.join('layout', '*'))),
        # 지도는 services/roport-pinky/maps/ 공용 파일(실로봇·관제 GUI와 같은 파일)을 설치한다
        ('share/' + package_name + '/maps',
            glob(os.path.join('..', '..', 'maps', 'airport_map*.yaml'))
            + glob(os.path.join('..', '..', 'maps', 'airport_map*.pgm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GATE CREW',
    maintainer_email='maintainer@example.com',
    description='RO:PORT demo space Gazebo world for Pinky Pro',
    license='LicenseRef-Proprietary',
    extras_require={'test': ['pytest']},
)
