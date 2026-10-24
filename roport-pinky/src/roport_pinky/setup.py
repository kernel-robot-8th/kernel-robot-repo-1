from setuptools import setup

setup(
    name="roport_pinky",
    version="0.1.0",
    packages=["roport_pinky"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/roport_pinky"]),
        ("share/roport_pinky", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="GATE CREW",
    maintainer_email="62865808+jsh0116@users.noreply.github.com",
    description="RO:PORT Pinky integration package",
    license="LicenseRef-Proprietary",
)
