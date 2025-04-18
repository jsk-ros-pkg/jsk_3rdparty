# ~/ros/catkin_ws/src/jsk_3rdparty/emotion_analyzer/setup.py

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['emotion_analyzer'],
    package_dir={'': 'src'}
)

setup(**d)
