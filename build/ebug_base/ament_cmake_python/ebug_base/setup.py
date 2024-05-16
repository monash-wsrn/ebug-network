from setuptools import find_packages
from setuptools import setup

setup(
    name='ebug_base',
    version='0.0.0',
    packages=find_packages(
        include=('ebug_base', 'ebug_base.*')),
)
