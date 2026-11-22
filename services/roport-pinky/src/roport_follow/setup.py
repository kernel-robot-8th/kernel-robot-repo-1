from setuptools import find_packages, setup

package_name = 'roport_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'roport_follow': ['web_static/*.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GATE CREW',
    maintainer_email='102033640+minjee92@users.noreply.github.com',
    description='RO:PORT smart cart card following with doll avoidance (laptop inference)',
    license='LicenseRef-Proprietary',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'follow_tool = roport_follow.tool:main',
            'nav2_doll = roport_follow.nav2_doll:main',
            'demo_runner = roport_follow.demo_runner:main',
        ],
    },
)
