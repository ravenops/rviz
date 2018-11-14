
from setuptools import setup

import os
os.path.dirname(os.path.abspath(__file__))

def readme():
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'README.md')) as f:
        return f.read()

setup( name         = 'rvnbag',
       author       = 'Iain Brookshaw',
       author_email = 'iain@raven-ops.com',
       version      = '0.1.0',
       url          = 'http://github.com/ravenops/rviz/scripts',
       license      = 'Copyright (c) 2018, Raven Ops Inc., All Rights Reserved',
       packages     = ['rvnbag'],
       install_requires=[
          'empy',
          'dbus-python', # MUST have `sudo apt-get install libdbus-1-dev` for this to work
          'rospkg'       # for some reason, this is seperate! (apt-get is prefered: http://wiki.ros.org/rospkg#Installing_rospkg)
       ],
       description  = 'Package playing back rosbags at our own pace via dbus',
       long_description_content_type="text/markdown",
       long_description = readme(),
       scripts  = ["rvnbag/scripts/rvnbag_play.py"],
       zip_safe = False )
