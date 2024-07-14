from setuptools import find_packages
from setuptools import setup

setup(
    name='autonoumous_navigation',
    version='1.0.0',
    packages=find_packages(
        include=('autonoumous_navigation', 'autonoumous_navigation.*')),
)
