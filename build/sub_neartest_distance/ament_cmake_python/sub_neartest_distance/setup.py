from setuptools import find_packages
from setuptools import setup

setup(
    name='sub_neartest_distance',
    version='1.0.0',
    packages=find_packages(
        include=('sub_neartest_distance', 'sub_neartest_distance.*')),
)
