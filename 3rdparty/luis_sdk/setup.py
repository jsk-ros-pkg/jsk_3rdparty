#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

pkgdir = {'': 'python%s' % sys.version_info[0]}
VERSION = '0.1'

d = generate_distutils_setup(
    # name="cognitive_luis",
    # version=VERSION,
    description='LUIS SDK for python',
    url='https://github.com/Microsoft/Cognitive-LUIS-Python',
    author='Ahmed El-Hinidy',
    author_email='t-ahelhi@microsoft.com',
    license='MIT',
    package_dir=pkgdir,
    packages=['luis_sdk'],
    # zip_safe=False
)

setup(**d)
