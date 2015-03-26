#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup( 
    packages=['octomap_to_ellipsoid'],
    package_dir={'': '.'}
)

setup(**d)
