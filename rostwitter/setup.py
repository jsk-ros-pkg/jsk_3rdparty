#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


d = generate_distutils_setup(
    packages=['rostwitter'],
    package_dir={'': 'python'},
    scripts=['scripts/tweet.py'],
)

setup(**d)
