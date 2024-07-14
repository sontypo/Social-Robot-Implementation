from setuptools import find_packages
from setuptools import setup

setup(
    name='pub_neartest_distance',
    version='1.0.0',
    packages=find_packages(
        include=('pub_neartest_distance', 'pub_neartest_distance.*')),
)
