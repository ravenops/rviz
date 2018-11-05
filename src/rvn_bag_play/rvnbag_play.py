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
class MessageContainer(object):
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
class BagReader(object):
    """ Responsible for loading the bag. Will yield a generator for all messages within a given time window """

    def __init__( self, bagfile ):
        """
        :params bagfile: str --- full path to the desired ROS bagfile
        :params delta:   rospy.Duration -- frame duration
        """
        try:
            self.bag   = rosbag.Bag(bagfile)
        except Exception as e:
            raise Exception("Unable to load bagfile from path '{}': {}".format(bagfile,e))

        self.reseek( 0 )
        self.clock_out = rospy.Time(0)


    def _reached_frame_end( self, t ):
        if t < self._frame_end:
            return False
        else:
            return True


    def reseek( self, seek_point ):
        """ stops the current itteration and resets the read loop criteria. input time is from start of bag not Unix time"""
        if type(seek_point) is float or type(seek_point) is int: 
            seek_point = seek_point + self.bag.get_start_time()
            seek_point = rospy.Time(seek_point)

        if rospy.Time(self.bag.get_end_time()) <= seek_point:
            return 1

        self._frame_end      = seek_point
        self._generator      = self.bag.read_messages(raw=True, start_time=seek_point, end_time=rospy.Time(self.bag.get_end_time()))
        return 0


    def get_next_frame( self, duration ):
        """ loop over all messages in bag. Loop start defined by 'reseek()' """
        if type(duration) is float:
            duration = rospy.Duration(duration)
        
        frame=[]
        self._frame_end = self._frame_end+duration
        self.clock_out = rospy.Time(0)

        for topic, raw_msg, t in self._generator:

            if self._reached_frame_end(t): 
                break
            else:
                frame.append( MessageContainer(topic, raw_msg=raw_msg, t=t) )
                if self.clock_out < frame[-1].time_of_bagging:
                    self.clock_out = frame[-1].time_of_bagging

        return frame        



# ----------------------------------------------------------------------------------------------------------------------
class PublicationControl(object):
    __doc__ = """
        <node>
            <interface name='{}'>
            <method name='kill'/>
            <method name='seek'>
                <arg type='d' name='at_time_seconds' direction='in'/>
                <arg type='u' name='seek_return'     direction='out'/>
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

    def __init__( self, bagfile,loop):

        self.exit_status     = ExitStatus.OK
        self._all_publishers = {}
        self._last_ros_clock_msg = None
        self._loop = loop

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
        self._clock_publisher = rospy.Publisher("/clock", Clock, queue_size=100)


    def _create_publisher( self, msg_cont ):

        try:
            new_pub = rospy.Publisher(msg_cont.topic, msg_cont.msg_class, queue_size=100) # RVN::FIX: queue_size=???)
        except Exception as e:
            self._set_terminate( ExitStatus.PUB_FAIL, "Cannot create publisher for '{}' topic: {}".format(msg_cont.topic,e))

        self._all_publishers[msg_cont.topic] = new_pub


    def _publish_msg( self, msg_cont ):

        # clock message is special and we handle it specially!
        if msg_cont.topic == "\clock": return
        
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
        if ex_str:
            print "{}".format(ex_str)


    def kill( self ):
        """ d-bus method to kill the player """
        self._set_terminate( ExitStatus.OK, "recieved 'kill' d-bus signal" )
        self._loop.quit()


    def seek( self, t ):
        """ d-bus method to terminate the read loop and seek to t seconds from start of bag(or as close as possible) """
        rospy.loginfo("recieved 'seek' signal. Seeking to {}".format(t))
        return self.bag_reader.reseek(t)
 

    def read( self, duration ):
        """
        d-bus method to read and publish the frame over duration. 
        generates a frame over the period of duration, which is then published to the ROS graph. 
        returns the last \clock message as seconds float. returns a -ve number if in error
        """
        dbg_start_read = time.time()
        out_clock = -1.0

        # ROS OK check
        if rospy.is_shutdown():
           print  "Error: tried to read frame when ROS is shutdown!"
           return -1.0
       
        try:
            frame = self.bag_reader.get_next_frame( duration )
        except Exception as e:
            print "{}".format(e)
            return out_clock
        
        clock_msg = Clock()
        clock_msg.clock = self.bag_reader.clock_out
        self._clock_publisher.publish(clock_msg)

        for msg in frame:
            # msg = frame[topic]
            self._publish_msg( msg )

        print "Have frame of {} messages. Clock = {}".format(len(frame),self.bag_reader.out_clock.to_sec())

        # return the clock via d-bus
        return out_clock.to_sec()



# ----------------------------------------------------------------------------------------------------------------------
def main():

    verbose = False
    bagfile = ""
    parser  = argparse.ArgumentParser() # RVN::FIX: usage=print_usage())
    parser.add_argument( '-b', '--bagfile', required=True, default="", help="complete bagfile path")
    # parser.add_argument( '-v', '--verbose', help="select verbose output", action='store_true')

    args = parser.parse_args()
    if args.bagfile: bagfile = args.bagfile
    # if args.verbose: verbose = True

    # We need to change the time -- ROS cannot use wall-time for this publication
    # RVN::FIX: check and confirm this is ok
    loop = GLib.MainLoop()

    try:
        pub = PublicationControl( bagfile, loop )
    except Exception as e:
        rospy.logerr("{}".format(e))
        return ExitStatus.PUB_FAIL

    rospy.loginfo("starting d-bus service...")
    session_bus = SessionBus()
    session_bus.publish( service_name, pub )
    loop.run()
    rospy.loginfo("...finished d-bus service")

    return pub.exit_status



# ----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    exit( main() )