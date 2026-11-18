from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["smart_device_protocol"], package_dir={"": "python"})

setup(**d)
