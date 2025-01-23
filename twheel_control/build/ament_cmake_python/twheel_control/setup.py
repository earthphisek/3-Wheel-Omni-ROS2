from setuptools import find_packages
from setuptools import setup

setup(
    name='twheel_control',
    version='0.0.0',
    packages=find_packages(
        include=('twheel_control', 'twheel_control.*')),
)
