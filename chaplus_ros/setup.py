#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['chaplus_ros'],
    # package_dir={'':'src'},
    scripts=['scripts/chaplus_ros.py']
)

setup(**d)
