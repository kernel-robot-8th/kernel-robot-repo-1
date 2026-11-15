from glob import glob

from setuptools import find_packages, setup

package_name = 'roport_pinky_line'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GATE CREW',
    maintainer_email='102033640+minjee92@users.noreply.github.com',
    description='RO:PORT Pinky IR line follower',
    license='LicenseRef-Proprietary',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'line_follower = roport_pinky_line.line_follower_node:main',
        ],
    },
)
