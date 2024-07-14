from setuptools import find_packages
from setuptools import setup

setup(
    name='object_detection',
    version='1.0.0',
    packages=find_packages(
        include=('object_detection', 'object_detection.*')),
)
