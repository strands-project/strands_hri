from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['strands_human_following'],
    scripts=['scripts/simple_follow_server.py'],
    package_dir={'': 'src'})

setup(**setup_args)
