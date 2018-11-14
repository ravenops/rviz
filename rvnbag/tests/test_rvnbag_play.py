#
"""
Raven[ player:testing ]

Iain Brookshaw
2018.10.30
Copyright (c) 2018, Raven Ops Inc., All Rights Reserved

Raven Rosbag player, using D-Bus to communicate to the RvnRviz renderer
Unit Tests
"""

import unittest
import os

import rospy
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

import subprocess as sub
import dbus

import rvnbag as rvn
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
float64bagfile = os.path.join(_THIS_DIR, "tests/testdata/float64.bag")


class TestMessageContainer( unittest.TestCase ):

    def setUp(self):
        self.bag = rospy.Bag(float64bagfile)

    def tearDown(self):
        self.bag.close()

    def test_deserialize( self ):

        for topic, raw, t in self.bag.read_messages( raw=True ):
            mc = rvn.MessageContainer( topic, raw, t)
            ds = mc.deserialize()
            self.assertTrue( ds.data, 42.0 )


    def test_msg_in_time_window( self ):

        for topic, raw, t in self.bag.read_messages( raw=True ):
            mc = rvn.MessageContainer( topic, raw, t )
            s = t + rospy.Duration(-0.01)
            e = t + rospy.Duration(0.01)
            self.assertTrue( mc.msg_in_time_window(s,e))


class TestBagReader( unittest.TestCase ):

    def setUp(self):
        self.br = rvn.BagReader(float64bagfile)

    # def tearDown(self):

    def test_get_next_frame( self ):

        # each 0.1 sec frame of a 10Hz topic should have one message

        duration = self.br.bag.get_end_time() - self.br.bag.get_start_time()
        count = duration / 0.1

        for dt in [0.1]*count:
            fr = self.br.get_next_frame( 0.1 )
            self.assertTrue(len(fr), 1)


class TestPublicationControl( unittest.TestCase ):

    def _sub_cb( self, msg ):
        self.frame.append(msg)

    def _clock_cb( self, msg ):
        self._last_clock = msg.clock
        self._frame_lock = False


    def setUp( self ):

        # RVN::NOTE: This is something of a hack, as testing seems to reset the env vars!
        env = dict(os.environ)
        env["PYTHONPATH"] = os.environ["PYTHONPATH"].split(":")[-1]
        run = "roscore > /dev/null && python " + float64bagfile
        self.cmd = sub.Popen( run, env=env, shell=True )

        # subscribe to the correct topic
        self.msg_sub = rospy.Subscriber("/test/float64", Float64, self._sub_cb)
        self.clk_sub = rospy.Subscriber("/clock",        Clock,   self._clock_cb)
        self.frame = None

        # connect to the session bus
        session_bus = dbus.SessionBus()
        self._frame_lock = True
        self._last_clock = -1

        try:
            self.proxy = bus.get_object(rvn.service_name, '/PublicationControl')
        except Exception as e:
            self.fail("Unable to get 'PublicationControl' at service {}: {}".format(rvn.service_name, e))

    def tearDown( self ):
        self.cmd.kill()


    def test_seek( self ):
        self.assertEqual( proxy.Seek(0), 0.0 )


    def test_read( self ):

        bg = rospy.Bag(float64bagfile)
        duration = bg.get_end_time() - bg.get_start_time()
        count = duration / 0.1

        for dt in [0.1]*count:

            # sumon a frame
            last_clock = self.br.Read( 0.1 )

            self._frame_lock = True
            while self._frame_lock:
                time.sleep(0.01)

            self.assertTrue(self._last_clock, last_clock)

            self.assertTrue(len(self.frame), 1)

