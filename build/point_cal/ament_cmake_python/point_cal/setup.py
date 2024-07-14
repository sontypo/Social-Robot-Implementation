from setuptools import find_packages
from setuptools import setup

setup(
    name='point_cal',
    version='1.0.0',
    packages=find_packages(
        include=('point_cal', 'point_cal.*')),
)
