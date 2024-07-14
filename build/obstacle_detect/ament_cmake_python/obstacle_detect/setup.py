from setuptools import find_packages
from setuptools import setup

setup(
    name='obstacle_detect',
    version='1.0.0',
    packages=find_packages(
        include=('obstacle_detect', 'obstacle_detect.*')),
)
