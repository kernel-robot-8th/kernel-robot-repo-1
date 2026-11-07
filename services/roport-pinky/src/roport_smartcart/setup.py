from glob import glob

from setuptools import find_packages, setup

package_name = 'roport_smartcart'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'IMPLEMENTATION_PLAN.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config/reference/2026-09-14',
         glob('config/reference/2026-09-14/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GATE CREW',
    maintainer_email='102033640+minjee92@users.noreply.github.com',
    description='RO:PORT smart cart (Pinky3) mission package',
    license='LicenseRef-Proprietary',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['navigate = roport_smartcart.navigate:main',
                                     'sim_sonar = roport_smartcart.sim_sonar:main',
                                     'sim_return_h1 = roport_smartcart.sim_return_h1:main']},
)
