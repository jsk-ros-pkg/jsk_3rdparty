from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


d = generate_distutils_setup(
    packages=['influxdb_store'],
    package_dir={'': 'python'}
)

setup(**d)
