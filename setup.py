from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ivr_assignment',
              'ivr_assignment.color',
              'ivr_assignment.control',
              'ivr_assignment.fusion',
              'ivr_assignment.sensor'],
    package_dir={'': 'src'})

setup(**setup_args)