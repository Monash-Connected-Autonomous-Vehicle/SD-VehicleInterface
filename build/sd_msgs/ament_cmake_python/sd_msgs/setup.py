from setuptools import find_packages
from setuptools import setup

setup(
    name='sd_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('sd_msgs', 'sd_msgs.*')),
)
