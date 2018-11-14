"""
Raven[ player:testing ]

Iain Brookshaw
2018.10.30
Copyright (c) 2018, Raven Ops Inc., All Rights Reserved

Unit Tests -- simple script to test running of the the rvnbag_play main functions -- test script will 
emulate the other end of the dbus
"""

import rvnbag as rvn
import os
import sys

from dbus         import SessionBus
from gi.repository  import GLib


_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
bagfile = sys.argv[1]

loop = GLib.MainLoop()
pc = rvn.PublicationControl(bagfile, loop)
session_bus = SessionBus()
loop.run()
