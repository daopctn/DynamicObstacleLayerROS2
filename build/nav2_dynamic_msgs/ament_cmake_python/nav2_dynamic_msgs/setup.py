from setuptools import find_packages
from setuptools import setup

setup(
    name='nav2_dynamic_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('nav2_dynamic_msgs', 'nav2_dynamic_msgs.*')),
)
