from setuptools import find_packages
from setuptools import setup

setup(
    name='irobot_create_common_bringup',
    version='3.0.4',
    packages=find_packages(
        include=('irobot_create_common_bringup', 'irobot_create_common_bringup.*')),
)
