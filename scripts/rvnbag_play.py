#! /usr/bin/env python2.7
"""
Raven[ player ]

Iain Brookshaw
Copyright (c) 2018, Raven Ops Inc., All Rights Reserved

Raven Rosbag player, using D-Bus to communicate to the RvnRviz renderer

"""

# General Python Imports
import os
import sys
import time
import argparse
import threading
from enum import Enum

# ROS Imports
import rosbag
import rospy
from rosgraph_msgs.msg import Clock

# D-Bus imports
from pydbus         import SessionBus
from gi.repository  import GLib
from pydbus.generic import signal

# define the dbus name and ROS publication node names
service_name  = 'com.ravenops.rviz.LockStep'
ros_node_name = 'rvn_bag_play'

class ExitStatus(Enum):
    OK           = 0
    BAD_DESERIAL = 3
    BAD_BAG      = 4
    PUB_FAIL     = 5



# ----------------------------------------------------------------------------------------------------------------------
class BagReader(object):
    """ Responsible for loading the bag. Will yield a generator for all messages within a given time window """

    def __init__( self, bagfile ):
        """
        :params bagfile: str --- full path to the desired ROS bagfile
        :params delta:   rospy.Duration -- frame duration
        """

        # start the bag loading thread
        self.event = threading.Event()
        self.event.clear()

        bag_load_thread   = threading.Thread(target=self.load_bagfile, args=(bagfile,))
        self._bag         = None
        bag_load_thread.start()

        self._latching_status = {}
        self.clock_out = rospy.Time(0)

        # set when msg w/ topic = /rvn/rviz/ctrl/preload_end is seen
        self.preload_end_time = None
        self.first_read = True

    def _reached_frame_end( self, t ):
        if t < self._frame_end:
            return False
        else:
            return True

    def load_bagfile( self, bagfilename ):
        """ seperate thread function for bag loading """
        try:
            self._bag = rosbag.Bag(bagfilename)
        except Exception as e:
            sys.exit("FATAL: exception encountered: unable to load bagfile from path '{}': reason = {}".format(bagfilename,e))
        except:
            sys.exit("FATAL: unexpected error: unable to load bagfile from path '{}': reason = {}".format(bagfilename,sys.exc_info()[0]))

        self.event.set()


    def wait_for_bag( self, timeout ):
        """ hold for bag to be ok or timeout exipires.. It timeout raises exception"""
        if not self.event.wait(timeout=timeout):
            raise Exception("Wait for bag timed out after {} seconds".format(timeout))


    def get_dt( self ):
        if self.clock_out.to_sec() == 0:
            return 0.0
        else:
            return self.clock_out.to_sec()-self._bag.get_start_time()


    def get_duration( self ):
        """returns the duration of the bag in seconds"""
        return self._bag.get_end_time() - self._bag.get_start_time()


    def get_preload_duration(self):
        if self.preload_end_time is None:
            return -1.0
        return self.preload_end_time - self._bag.get_start_time()

    def is_latched(self, topic):
        """get the latching status of this topic"""
        if topic not in self._latching_status:
            return False
        else:
            return self._latching_status[topic]


    def _header_callback( self, topic, datatype, md5sum, msg_def, header ):
        """ Callback to get the topic connection header for this msg. This is so we can store the latched topics """
        # the /tf_static topic is a special case -- it is ALWAYS latched!
        if topic == "/tf_static":
            self._latching_status[topic] = True
            return True

        # if not specified, default
        if "latching" not in header:
            self._latching_status[topic] = False
            return True

        # set the value for all other topics
        if topic not in self._latching_status:
            self._latching_status[topic] = header["latching"]

        return True


    def initialize_generator( self ):
        """ stops the current itteration and resets the read loop criteria. input time is from start of bag not Unix time"""

        self._frame_end = rospy.Time(self._bag.get_start_time())

        self._generator = self._bag.read_messages(
            raw        = True,
            connection_filter = self._header_callback )

        # reset clock
        self.clock_out = rospy.Time(self._bag.get_start_time())

    def get_next_frame( self, duration, timeout=30.0 ):

        if type(duration) is float:
            duration = rospy.Duration(duration)

        self._frame_end = self._frame_end+duration
        self.clock_out  = rospy.Time(0)

        for topic, raw_msg, t in self._generator:
            if self.first_read:
                if topic != "/rvn/rviz/ctrl/preload_start":
                    raise Exception("first message of bag must be preload start")
                self.first_read = False

            if self.clock_out < t:
                self.clock_out = t

            if topic == "/rvn/rviz/ctrl/preload_end":
                self.preload_end_time = self.clock_out.to_sec()

            yield {"topic" : topic, "raw_msg" : raw_msg, "t": t}

            if self._reached_frame_end(t):
                break
        return


# ----------------------------------------------------------------------------------------------------------------------
class PublicationControl(object):
    __doc__ = """
        <node>
            <interface name='{}'>
            <method name='kill'/>
            <method name='bag_duration'>
                <arg type='d' name='timeout'                 direction='in' />
                <arg type='d' name='bag_duration_return'     direction='out'/>
            </method>
            <method name='preload_duration'>
                <arg type='d' name='preload_duration_return'     direction='out'/>
            </method>
            <method name='read'>
                <arg type='d' name='duration_seconds'   direction='in' />
                <arg type='d' name='timeout'            direction='in' />
                <arg type='d' name='last_ros_clock_msg' direction='out'/>
            </method>
            <property   name="current_time" type="d" access="read">
            <annotation name="org.freedesktop.DBus.Property.EmitsChangedSignal" value="true"/>
            </property>
            </interface>
        </node>
    """.format(service_name)

    def __init__( self, bagfile, loop):

        self.exit_status     = ExitStatus.OK
        self._all_publishers = {}
        self._last_ros_clock_msg = None
        self._loop = loop #used to issue quit command to kill

        try:
            rospy.init_node( ros_node_name, anonymous=True )
        except Exception as e:
            raise Exception("Unable to start the publication node: {}".format(e))

        # This publication must use simulation time!
        rospy.set_param("use_sim_time", True)

        rospy.loginfo("Opening ROS bag at '{}' for reading...".format(bagfile))
        try:
            self.bag_reader = BagReader( bagfile )
        except Exception as e:
            self._set_terminate( ExitStatus.BAD_BAG, "Unable to create BagReader: {}".format(e))
        rospy.loginfo("...Done")

        # create clock publisher
        self._clock_publisher = rospy.Publisher("/clock", Clock, queue_size=1, tcp_nodelay=True)


    def _create_publisher( self, topic, msg_class ):
        try:
            new_pub = rospy.Publisher(
                topic,
                msg_class,
                latch      = self.bag_reader.is_latched(topic),
                queue_size = 16,
                tcp_nodelay = True,
            )
        except Exception as e:
            self._set_terminate( ExitStatus.PUB_FAIL, "Cannot create publisher for '{}' topic: {}".format(topic,e))
            return

        self._all_publishers[topic] = new_pub


    def _publish_msg( self, msg ):

        # clock message is special and we handle it specially!
        if msg["topic"] == "/clock": return

        if msg["topic"] not in self._all_publishers:
            self._create_publisher(msg["topic"],msg["raw_msg"][-1])

        pmsg = rospy.msg.AnyMsg()
        pmsg.deserialize(msg["raw_msg"][1])
        self._all_publishers[msg["topic"]].publish(pmsg)


    def _set_terminate( self, code, ex_str="" ):
        """sets termination flag, error code status and raises exception (if set)"""
        self.exit_status = code
        if ex_str:
            print "{}".format(ex_str)


    def kill( self ):
        """ d-bus method to kill the player """
        self._set_terminate( ExitStatus.OK, "recieved 'kill' d-bus signal" )
        self._loop.quit()

    def read( self, duration, timeout ):
        """
        d-bus method to read and publish the frame over duration.
        generates a frame over the period of duration, which is then published to the ROS graph.
        returns the last \clock message as seconds float. returns a -ve number if in error

        RVN::NB: This will not do anything if you have not already 'seeked' prior to calling
        this method. This ensures that an initial re-seek back to 0 does not befoul the
        tf tree.
        """
        self.bag_reader.wait_for_bag(timeout)

        old_clock = self.bag_reader.clock_out
        clock_dt  = rospy.Duration(0.01) # RVN:FIX: This is Hard Coded!

        # ROS OK check
        if rospy.is_shutdown():
           rospy.logerr("Error: tried to read frame when ROS is shutdown!")
           return -1.0

        # get the frame
        try:
            # publish the message
            for msg in self.bag_reader.get_next_frame( duration, timeout=timeout ):
                self._publish_msg( msg )

                # return the clock via d-bus and possibly publish to system
                if self.bag_reader.clock_out - old_clock > clock_dt:
                    old_clock = self.bag_reader.clock_out
                    self._clock_publisher.publish(self.bag_reader.clock_out)

        except Exception as e:
            rospy.logerr("unable to get next frame: %r" % (e))
            return self.bag_reader.clock_out.to_sec()

        return old_clock.to_sec() - self.bag_reader._bag.get_start_time()


    def bag_duration(self,timeout):
        self.bag_reader.wait_for_bag(timeout)
        self.bag_reader.initialize_generator()
        return self.bag_reader.get_duration()

    def preload_duration(self):
        return self.bag_reader.get_preload_duration()
# ----------------------------------------------------------------------------------------------------------------------

def main():
    rospy.loginfo("Starting Raven[ops] ROS Bag Player")

    verbose = False
    bagfile = ""
    parser  = argparse.ArgumentParser()
    parser.add_argument( '-b', '--bagfile', required=True, default="",    help="complete bagfile path")
    parser.add_argument( '-v', '--verbose',                default=False, help="select verbose output", action='store_true')
    parser.add_argument("args",nargs='+')
    args = parser.parse_args()
    if args.bagfile: bagfile = args.bagfile
    if args.verbose: verbose = True # RVN::TODO: Not connected


    rospy.loginfo("Getting main Glib loop object")
    try:
        loop = GLib.MainLoop()
    except Exception as e:
        rospy.logerr("Unable to create GLib.MainLoop!")
        return ExitStatus.PUB_FAIL

    rospy.loginfo("Creating Publication Control Object")
    try:
        pub = PublicationControl( bagfile, loop )
    except Exception as e:
        rospy.logerr("{}".format(e))
        return ExitStatus.PUB_FAIL

    rospy.loginfo( "starting d-bus service...")
    try:
        session_bus = SessionBus()
    except Exception as e:
        rospy.logerr("Unable to create the session dbus")
        return ExitStatus.PUB_FAIL

    rospy.loginfo("created the session d-bus object")

    try:
        session_bus.publish( service_name, pub )
    except Exception as e:
        rospy.logerr("Unable to set up session dbus: {}".format(e))
        return ExitStatus.PUB_FAIL

    rospy.loginfo("published to the '{}' dbus ".format(service_name))
    rospy.loginfo("running the main loop...")

    loop.run()
    rospy.loginfo("...finished d-bus service")

    return pub.exit_status



# ----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    exit( main() )
