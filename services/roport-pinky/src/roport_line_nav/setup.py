from glob import glob

from setuptools import find_packages, setup

package_name = 'roport_line_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GATE CREW',
    maintainer_email='102033640+minjee92@users.noreply.github.com',
    description='RO:PORT IR line node-route navigation',
    license='LicenseRef-Proprietary',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'line_nav = roport_line_nav.line_nav_node:main',
            'cmd_vel_guard = roport_line_nav.cmd_vel_guard:main',
            'status_indicator = roport_line_nav.status_indicator:main',
        ],
    },
)
