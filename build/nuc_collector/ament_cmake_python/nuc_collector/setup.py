from setuptools import find_packages
from setuptools import setup

setup(
    name='nuc_collector',
    version='0.0.0',
    packages=find_packages(
        include=('nuc_collector', 'nuc_collector.*')),
)
