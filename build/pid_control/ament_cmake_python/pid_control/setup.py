from setuptools import find_packages
from setuptools import setup

setup(
    name='pid_control',
    version='1.0.0',
    packages=find_packages(
        include=('pid_control', 'pid_control.*')),
)
