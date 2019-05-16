## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['piksi_multi_rtk_ros'],
    scripts=['bin/geodetic_survey', 'bin/piksi_multi', 'bin/gnss_fix_degradation'],
    package_dir={'': 'src'})

setup(**setup_args)
