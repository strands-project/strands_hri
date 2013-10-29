#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   #  don't do this unless you want a globally visible script
   # scripts=['bin/myscript'], 
   packages=['sound_to_lights','peak_detect','pulseaudio'],
   package_dir={'sound_to_lights': 'src', 'peak_detect': 'src/sound_to_lights/contrib', 'pulseaudio': 'src/sound_to_lights/contrib/pulse'}
)

setup(**d)
