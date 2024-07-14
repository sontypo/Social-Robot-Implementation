from setuptools import find_packages
from setuptools import setup

setup(
    name='run_sensor',
    version='1.0.0',
    packages=find_packages(
        include=('run_sensor', 'run_sensor.*')),
)
