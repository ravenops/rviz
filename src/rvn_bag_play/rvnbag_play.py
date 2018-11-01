#! /usr/bin/env python2.7
"""
Raven[ player ]

Iain Brookshaw
2018.10.30
Copyright (c) 2018, Raven Ops Inc., All Rights Reserved

Raven Rosbag player, using D-Bus to communicate to the RvnRviz renderer

RVN::FIX: This has to become a ROS Package, because ROS!!!
"""

# General Python Imports
import argparse
import os
import time
import signal
import logging as log
from enum import Enum

# ROS and RVN Imports
import rosbag
import rospy
import rvn_events.tools as tools
import rvnbag_parts as rvnbag

# D-Bus imports
from pydbus         import SessionBus
from gi.repository  import GLib
from pydbus.generic import signal

service_name = 'com.ravenops.rviz.LockStep'
loop = GLib.MainLoop()

class ExitStatus(Enum):
    OK           = 0
    BAD_DESERIAL = 3
    BAD_BAG      = 4
    PUB_FAIL     = 5



# ----------------------------------------------------------------------------------------------------------------------
class MessageContainer:
    """
    It is much more convenient to have a message holder class than try and untangle the 
    tupple supplied
    """
    def __init__(self, topic, raw_msg=None, t=rospy.Time()):
        self.topic           = topic
        self.msg_class       = raw_msg[-1]
        self.raw_msg_data    = raw_msg[1]
        self.time_of_bagging = t


    def deserialize( self ):
        """ create an instance of the message class and return it """
        if not self.raw_msg_data or not self.msg_class:
            raise Exception("either raw msg or class are invalid.\n\traw_msg: {}\n\tmsg_class: {}".format(self.raw_msg_data,self.msg_class))
            return

        try:
            cls = self.msg_class()
        except Exception as e:
            raise Exception("Unable to create an instance of the message class: {}".format(e))

        try:
            msg = cls.deserialize(self.raw_msg_data)
        except Exception as e:
            raise Exception("Unable to deserialize raw message data into message: {}".format(e))

        return msg


    def msg_in_time_window( self, start, end ):
        """ returns t/f if the message bagging timestamp within the window """
        if start <= self.time_of_bagging and self.time_of_bagging <= end:
            return True
        else:
            return False


# ----------------------------------------------------------------------------------------------------------------------
class Frame:
    """ All the messages in a single frame -- storage struct """

    def __init__( self, start, end ):
        self.all_msgs   = [] # all message containers
        self.last_clock = None
        self.start      = start
        self.end        = end

    def add_msg(self, topic, raw_msg, t):
        msg_cont = MessageContainer(topic, raw_msg=raw_msg, t=t)
        self.all_msgs.append(msg_cont)
        if topic == "/clock":
            self.last_clock = msg_cont




# ----------------------------------------------------------------------------------------------------------------------
class BagReader:
    """ Responsible for loading the bag. Will yield a generator for all messages within a given time window """

    def __init__( self, bagfile ):
        """
        :params bagfile: str --- full path to the desired ROS bagfile
        """
        self.bag   = tools.load_bag(bagfile)
        self.now   = rospy.Time(self.bag.get_start_time())

    def get_the_next_frame( self, delta ):
        """
        returns the frame defined by now + delta 
        :params delta:   rospy.Duration -- frame duration
        """
        # RVN::TODO: Although this is a generator, it probably isn't all that efficient, but this
        # is intended exclusively for the clips, which are short bags
        # RVN::FIX: need to be able to stop, start this... 
        frame = Frame(self.now, self.now+delta)
        for topic, raw_msg, t in self.bag.read_messages(raw=True, start_time=frame_start, end_time=frame_end):
            frame.add_msg(topic, raw_msg, t)

        self.now = frame.end
        return frame

    def seek( self, seek_point ):
        """resets the 'now' to seek_point, abandoning the last search"""
        # RVN::FIX: This is in conflict with what we planned -- need to reconcile the loop break with read_messages generator
        self.now = seek_point


# ----------------------------------------------------------------------------------------------------------------------
class PublicationControl:
    __doc__ = """
        <node>
        <interface name='{}'>
            <method name='kill'/>
            <method name='seek'>
                <arg type='d' name='at_time_seconds' direction='in'/>
            </method>
            <method name='read'>
                <arg type='d' name='duration_seconds'   direction='in'/>
                <arg type='d' name='last_ros_clock_msg' direction='out'/>
            </method>
            <property name="current_time" type="d" access="read">
            <annotation name="org.freedesktop.DBus.Property.EmitsChangedSignal" value="true"/>                          # RVN::FIX: Implement
            </property>
        </interface>
        </node>
    """.format(service_name)


    def __init__( self, bagfile):

        self.delta           = rospy.Duration(0)
        self.terminate       = False  # RVN::NB: This is critical, as this flag is used to terminate the main loop
        self.exit_status     = ExitStatus.OK
        self._all_publishers = {}
        self._last_ros_clock_msg = None

        rospy.loginfo("Opening ROS bag at '{}' for reading...".format(bagfile))
        try:
            self.bag_reader = rvnbag.BagReader( bagfile, self.delta )
        except Exception as e:
            self._set_terminate( ExitStatus.BAD_BAG, "Unable to create BagReader: {}".format(e))
        rospy.loginfo("...Done")


    def _create_publisher( self, msg_cont ):

        try:
            new_pub = rospy.Publisher(msg_cont.topic, msg_cont.msg_class, queue_size=100) # RVN::FIX: queue_size=???)
        except Exception as e:
            self._set_terminate( ExitStatus.PUB_FAIL, "Cannot create publisher for '{}' topic: {}".format(msg_cont.topic,e))

        self._all_publishers[msg_cont.topic] = new_pub


    def _publish_msg( self, msg_cont ):
        
        if msg_cont.topic not in self._all_publishers:
            self._create_publisher(msg_cont)

        try:
            msg = msg_cont.deserialize()
        except Exception as e:
            self._set_terminate( ExitStatus.BAD_DESERIAL, "Unable to publish msg to '{}' topic, cannot deserialize: {}".format(msg_cont.topic,e))

        self._all_publishers[msg_cont.topic].publish(msg)
        # RVN::FIX: Publish the bagged time as "/clock"


    def _set_terminate( self, code, ex_str="" ):
        """sets termination flag, error code status and raises exception (if set)"""
        self.exit_status = code
        self.terminate   = True
        if ex_str:
            raise Exception(ex_str)


    def kill( self ):
        """ d-bus method to kill the player """
       self._set_terminate( ExitStatus.OK, "recieved 'kill' d-bus signal" )


    def seek( self, t ):
        """ d-bus method to terminate the read loop and seek to t (or as close as possible) """
        rospy.loginfo("recieved 'seek' signal. Seeking to {}".format(t))
        ## RVN::FIX: Implement the seek -- is this a null concept with the generator??


    def read( self, duration ):
        """
        d-bus method to read and publish the frame over duration. 
        generates a frame over the period of duration, which is then published to the ROS graph. 
        returns the last \clock message as seconds float
        """
        rospy.log("recieved the 'read' d-bus signal. Publishing frame of {} seconds".format(duration))
        frame_duration = rospy.Duration(duration)
        frame = self.bag_reader.get_the_next_frame( frame_duration )
        
        for msg in frame.msgs:
            self._publish_msg(msg)

        rospy_clock = frame.last_clock.deserialize()
        return rospy_clock.to_secs()



# ----------------------------------------------------------------------------------------------------------------------
def main():

    verbose = False
    bagfile = ""
    parser  = argparse.ArgumentParser() #RVN::FIX: usage=print_usage())
    parser.add_argument( '-b', '--bagfile', required=True, default="", help="complete bagfile path")
    parser.add_argument( '-v', '--verbose', help="select verbose output", action='store_true')

    args = parser.parse_args()
    if args.bagfile: bagfile = args.bagfile
    if args.verbose: verbose = True

    # We need to change the time -- ROS cannot use wall-time for this publication
    # RVN::FIX: check and confirm this is ok
    rospy.set_param("use_sim_time", True)

    # initalize this as a ROS Node
    rospy.init_node("rvnbag_play", anonymous=True)

    # get the bag reader
    rospy.loginfo("loading bagfile '{}'".format(bagfile))
    try:
        pub = PublicationControl( bagfile )
    except Exception as e:
        rospy.logerr("{}".format(e))
        return ExitStatus.PUB_FAIL
    rospy.loginfo("....DONE!")

    # Holding Loop -- hold until publication control says terminate or ROS says shutdown
    rospy.loginfo("Awaiting first frame d-bus msg...")
    while not pub.terminate:
        time.sleep(0.25)
        if rospy.is_shutdown():
            rospy.logwarn("ROS shutdown detected")
            break

    return pub.exit_status



# ----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    ret = main()
    exit(ret)